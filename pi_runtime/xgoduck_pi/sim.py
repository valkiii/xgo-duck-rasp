"""A simulated servo bus and IMU, for running and testing everything without hardware.

SimSerial behaves like a pyserial port with HD-1910 servos on the other end: it
answers ping, read, write, sync read and sync write, and servos with torque on move
towards their goal with a short lag. With echo=True it also echoes every
transmitted byte, as some half-duplex adapters do.
"""
from __future__ import annotations

import random
import threading
import time

from . import scs_bus as scs

DEFAULT_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)


class SimServo:
    def __init__(self, servo_id: int, pos: int = 2047) -> None:
        self.id = servo_id
        self.pos = float(pos)
        self.goal = float(pos)
        self.speed = 0.0          # steps per second
        self.torque = False
        self.lock = 1
        self.temp_kp, self.temp_kd = 32, 0
        self.perm_kp, self.perm_kd = 32, 0

    def step(self, dt: float) -> None:
        if not self.torque:
            self.speed = 0.0
            return
        old = self.pos
        self.pos += (self.goal - self.pos) * min(1.0, dt / 0.04)
        self.speed = (self.pos - old) / dt if dt > 0 else 0.0

    def registers(self, addr: int, length: int) -> bytes:
        pos = int(round(self.pos)) & 0xFFFF
        speed = int(round(abs(self.speed) / 50.0)) & 0x7FFF
        if self.speed < 0:
            speed |= 0x8000
        regs = {scs.ADDR_PRESENT: pos & 0xFF, scs.ADDR_PRESENT + 1: pos >> 8,
                scs.ADDR_PRESENT + 2: speed & 0xFF, scs.ADDR_PRESENT + 3: speed >> 8,
                scs.ADDR_PRESENT + 4: 0, scs.ADDR_PRESENT + 5: 0,
                scs.ADDR_TEMP_KP: self.temp_kp, scs.ADDR_TEMP_KD: self.temp_kd,
                scs.ADDR_ID: self.id, scs.ADDR_TORQUE: int(self.torque),
                scs.ADDR_PERM_KP_KD: self.perm_kp, scs.ADDR_PERM_KP_KD + 1: self.perm_kd}
        return bytes(regs.get(addr + i, 0) for i in range(length))

    def write(self, addr: int, data: bytes) -> None:
        for i, value in enumerate(data):
            a = addr + i
            if a == scs.ADDR_TORQUE:
                self.torque = bool(value)
            elif a == scs.ADDR_TEMP_KP:
                self.temp_kp = value
            elif a == scs.ADDR_TEMP_KD:
                self.temp_kd = value
            elif a == scs.ADDR_UNLOCK:
                self.lock = value
            elif a == scs.ADDR_ID:
                self.id = value
            elif a == scs.ADDR_PERM_KP_KD:
                self.perm_kp = value
            elif a == scs.ADDR_PERM_KP_KD + 1:
                self.perm_kd = value
        if addr <= scs.ADDR_GOAL_POS < addr + len(data) - 1:
            k = scs.ADDR_GOAL_POS - addr
            self.goal = float(data[k] | (data[k + 1] << 8))


def status(servo_id: int, params: bytes = b'', error: int = 0) -> bytes:
    body = bytes((servo_id, len(params) + 2, error)) + params
    return b'\xff\xff' + body + bytes((scs.checksum(body),))


class SimSerial:
    """Minimal pyserial stand-in: write/read/in_waiting/flush/timeout/close."""

    def __init__(self, servos=None, echo: bool = False, zero_pos=None, default_deg=None) -> None:
        if servos is None:
            servos = []
            for n, servo_id in enumerate(DEFAULT_IDS):
                zero = zero_pos[n] if zero_pos else 2047
                deg = default_deg[n] if default_deg else 0.0
                servos.append(SimServo(servo_id, zero - deg / 360.0 * 4095.0))  # all signs are -1
        self.servos = list(servos)
        self.echo = echo
        self.timeout = 0.0
        self.is_open = True
        self.fail = False           # set True to emulate a USB disconnect
        self.written = []           # every packet the host sent, for tests
        self._rx = bytearray()
        self._tx = bytearray()
        self._cv = threading.Condition()
        self._last = time.monotonic()

    # pyserial API --------------------------------------------------------
    def _check(self) -> None:
        if self.fail or not self.is_open:
            raise OSError('device disconnected')

    @property
    def in_waiting(self) -> int:
        self._check()
        with self._cv:
            return len(self._rx)

    def read(self, size: int = 1) -> bytes:
        self._check()
        deadline = time.monotonic() + (self.timeout or 0)
        with self._cv:
            while len(self._rx) < size:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cv.wait(remaining)
            data = bytes(self._rx[:size])
            del self._rx[:size]
            return data

    def write(self, data: bytes) -> int:
        self._check()
        with self._cv:
            if self.echo:
                self._rx += data
            self._tx += data
            self._process()
            self._cv.notify_all()
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        with self._cv:
            self._rx.clear()

    def close(self) -> None:
        self.is_open = False

    # bus emulation ---------------------------------------------------------
    def find(self, servo_id: int):
        return next((s for s in self.servos if s.id == servo_id), None)

    def _advance(self) -> None:
        now = time.monotonic()
        dt, self._last = now - self._last, now
        for s in self.servos:
            s.step(dt)

    def _process(self) -> None:
        while True:
            start = self._tx.find(b'\xff\xff')
            if start < 0 or len(self._tx) < start + 4:
                return
            length = self._tx[start + 3]
            end = start + 4 + length
            if len(self._tx) < end:
                return
            pkt = bytes(self._tx[start:end])
            del self._tx[:end]
            body = pkt[2:-1]
            if scs.checksum(body) != pkt[-1]:
                continue
            self.written.append(pkt)
            self._execute(pkt[2], pkt[4], pkt[5:-1])

    def _execute(self, target: int, inst: int, params: bytes) -> None:
        self._advance()
        broadcast = target == scs.BROADCAST
        if inst == scs.INST_PING:
            s = self.find(target)
            if s:
                self._rx += status(s.id)
        elif inst == scs.INST_READ:
            s = self.find(target)
            if s:
                self._rx += status(s.id, s.registers(params[0], params[1]))
        elif inst == scs.INST_WRITE:
            for s in (self.servos if broadcast else [self.find(target)]):
                if s:
                    s.write(params[0], params[1:])
                    if not broadcast:
                        self._rx += status(s.id)
        elif inst == scs.INST_SYNC_WRITE:
            addr, n = params[0], params[1]
            for k in range(2, len(params), n + 1):
                s = self.find(params[k])
                if s:
                    s.write(addr, params[k + 1:k + 1 + n])
        elif inst == scs.INST_SYNC_READ:
            addr, n = params[0], params[1]
            for servo_id in params[2:]:
                s = self.find(servo_id)
                if s:
                    self._rx += status(s.id, s.registers(addr, n))


class SimImu:
    """Upright and still, with a little sensor noise."""

    def __init__(self, noise: float = 30.0) -> None:
        self.ok = False
        self.noise = noise
        self.acc_raw = [0, 0, -16384]
        self.gyro_raw = [0, 0, 0]
        self.last_error = ''

    def init(self) -> bool:
        self.ok = True
        return True

    def update(self) -> bool:
        n = self.noise
        self.acc_raw = [int(random.gauss(0, n)), int(random.gauss(0, n)), -16384 + int(random.gauss(0, n))]
        self.gyro_raw = [int(random.gauss(0, n / 3)) for _ in range(3)]
        self.ok = True
        return True

    def close(self) -> None:
        pass
