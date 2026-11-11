"""Feetech SCS/STS serial-bus protocol for the HD-1910 servos.

Port of sketch/scs_bus.h from LuwuDynamics/xgoduck_runtime_arduino. Packet layout,
register addresses and the reply acceptance rules match the firmware, so the
host-side controller sees the same feedback it would get from the UNO Q.

The URT-2 (or any auto-direction half-duplex adapter) is opened as a normal serial
port. Some adapters echo every transmitted byte back on RX; that echo is detected
at start-up and stripped before parsing.
"""
from __future__ import annotations

import time
from typing import Callable, Iterable, NamedTuple, Optional

BROADCAST = 0xFE
INST_PING, INST_READ, INST_WRITE = 0x01, 0x02, 0x03
INST_SYNC_READ, INST_SYNC_WRITE = 0x82, 0x83

# duck_config.h
ADDR_ID = 0x05
ADDR_PERM_KP_KD = 0x15
ADDR_TORQUE = 40
ADDR_GOAL_POS = 0x2A
ADDR_TEMP_KP = 50
ADDR_TEMP_KD = 51
ADDR_UNLOCK = 0x37
ADDR_PRESENT = 0x38
PRESENT_LEN = 6
# Present voltage (0.1 V units) and present temperature (deg C) follow the 6-byte
# position/speed/load block directly: SMS_STS_PRESENT_VOLTAGE = 62, _TEMPERATURE = 63 in
# the Feetech SDK (same for SCSCL), i.e. one 2-byte read at 0x3E gives both.
ADDR_VOLTAGE = 0x3E
ADDR_TEMPERATURE = 0x3F
VOLT_TEMP_LEN = 2
TEMP_KP_RUN, TEMP_KD_RUN, TEMP_KP_MOUTH = 6, 20, 10
MOUTH_MOTOR_ID = 34
POS_MIN, POS_MAX = 0, 4095

# Reply LEN values the firmware parser accepts (status, 2-byte read, 6-byte read, ...).
REPLY_LENGTHS = frozenset((0x02, 0x04, 0x08, 0x09, 0x0B))


def checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def make_packet(servo_id: int, instruction: int, params: bytes = b'') -> bytes:
    body = bytes((servo_id & 0xFF, len(params) + 2, instruction)) + bytes(params)
    return b'\xff\xff' + body + bytes((checksum(body),))


def sat_pos(value: int) -> int:
    return POS_MIN if value < POS_MIN else POS_MAX if value > POS_MAX else int(value)


class Reply(NamedTuple):
    servo_id: int
    length: int      # the LEN byte: error + params + checksum
    error: int
    params: bytes


class ReplyParser:
    """Streaming status-packet parser with the firmware's state machine."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._state = 0
        self._id = 0
        self._len = 0
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[Reply]:
        out = []
        for b in data:
            st = self._state
            if st == 0:
                if b == 0xFF:
                    self._state = 1
            elif st == 1:
                self._state = 2 if b == 0xFF else 0
            elif st == 2:
                # Deviation from the firmware: 0xFF is never a servo ID, so a run of
                # FF bytes stays in the header instead of swallowing the next packet.
                if b != 0xFF:
                    self._id = b
                    self._state = 3
            elif st == 3:
                if b in REPLY_LENGTHS:
                    self._len = b
                    self._buf = bytearray()
                    self._state = 4
                else:
                    self._state = 0
            else:
                self._buf.append(b)
                if len(self._buf) == self._len:
                    self._state = 0
                    body = bytes((self._id, self._len)) + bytes(self._buf[:-1])
                    if checksum(body) == self._buf[-1]:
                        out.append(Reply(self._id, self._len, self._buf[0], bytes(self._buf[1:-1])))
        return out


def word(lo: int, hi: int) -> int:
    """hostWord(): little-endian int16."""
    v = lo | (hi << 8)
    return v - 0x10000 if v & 0x8000 else v


def speed_sign_magnitude(lo: int, hi: int) -> int:
    """hostVelSignMag(): bit 15 is a direction flag, not two's complement."""
    raw = lo | (hi << 8)
    speed = raw & 0x7FFF
    return -speed if raw & 0x8000 else speed


class ScsBus:
    """Packet-level access to one serial port. Not thread-safe; one owner thread."""

    def __init__(self, port, echo: Optional[bool] = None) -> None:
        # `port` is a pyserial Serial (or the simulator) opened at the bus baud rate.
        self.port = port
        self.port.timeout = 0.0005
        self.parser = ReplyParser()
        self.echo = bool(echo)
        self._skip = bytearray()  # transmitted bytes whose echo has not been seen yet

    # ---- transmit -------------------------------------------------------
    def write(self, packet: bytes) -> None:
        self.port.write(packet)
        if self.echo:
            self._skip += packet

    def flush(self) -> None:
        self.port.flush()

    def write_buf(self, servo_id: int, instruction: int, params: bytes = b'') -> None:
        self.write(make_packet(servo_id, instruction, params))

    def write_byte(self, servo_id: int, addr: int, value: int) -> None:
        self.write_buf(servo_id, INST_WRITE, bytes((addr, value & 0xFF)))

    def set_torque(self, enabled: bool) -> None:
        self.write_byte(BROADCAST, ADDR_TORQUE, 1 if enabled else 0)
        self.flush()

    def set_temp_kp_kd(self, kp: int, kd: int) -> None:
        self.write_byte(BROADCAST, ADDR_TEMP_KP, kp)
        self.flush()
        self.write_byte(BROADCAST, ADDR_TEMP_KD, kd)
        self.flush()

    def set_run_kp_kd(self) -> None:
        self.set_temp_kp_kd(TEMP_KP_RUN, TEMP_KD_RUN)
        self.write_byte(MOUTH_MOTOR_ID, ADDR_TEMP_KP, TEMP_KP_MOUTH)
        self.flush()

    def sync_write_goals(self, ids: Iterable[int], raw_positions: Iterable[int]) -> None:
        params = bytearray((ADDR_GOAL_POS, 6))
        for servo_id, pos in zip(ids, raw_positions):
            pos = sat_pos(pos)
            params += bytes((servo_id, pos & 0xFF, pos >> 8, 0, 0, 0, 0))
        self.write_buf(BROADCAST, INST_SYNC_WRITE, bytes(params))

    def write_single_goal_raw(self, servo_id: int, raw: int) -> None:
        raw = sat_pos(raw)
        self.write_buf(servo_id, INST_WRITE, bytes((ADDR_GOAL_POS, raw & 0xFF, raw >> 8, 0, 0, 0, 0)))

    def sync_read(self, ids: Iterable[int], addr: int, length: int) -> None:
        self.write_buf(BROADCAST, INST_SYNC_READ, bytes((addr, length, *ids)))

    def read(self, servo_id: int, addr: int, length: int) -> None:
        self.write_buf(servo_id, INST_READ, bytes((addr, length)))

    def ping(self, servo_id: int) -> None:
        self.write_buf(servo_id, INST_PING)

    def unlock(self, servo_id: int) -> None:
        self.write_byte(servo_id, ADDR_UNLOCK, 0)
        self.flush()

    def set_id(self, servo_id: int, new_id: int) -> None:
        self.write_byte(servo_id, ADDR_ID, new_id)
        self.flush()

    def set_perm_kp_kd(self, servo_id: int, kp: int, kd: int) -> None:
        self.write_buf(servo_id, INST_WRITE, bytes((ADDR_PERM_KP_KD, kp & 0xFF, kd & 0xFF)))
        self.flush()

    # ---- receive --------------------------------------------------------
    def _strip_echo(self, data: bytes) -> bytes:
        if not self._skip or not data:
            return data
        n = min(len(data), len(self._skip))
        k = 0
        while k < n and data[k] == self._skip[k]:
            k += 1
        if k == n:
            del self._skip[:n]
            return data[n:]
        # The adapter did not echo what we expected: stop skipping and parse the rest.
        self._skip.clear()
        return data[k:]

    def _read_some(self) -> bytes:
        waiting = self.port.in_waiting
        return self._strip_echo(self.port.read(waiting if waiting else 1))

    def drain(self) -> None:
        """drainServoRx(): discard everything received so far (echo bookkeeping kept)."""
        while True:
            waiting = self.port.in_waiting
            if not waiting:
                break
            self._strip_echo(self.port.read(waiting))
        self.parser.reset()

    def collect(self, seconds: float, handle: Callable[[Reply], None],
                done: Optional[Callable[[], bool]] = None) -> None:
        """Parse replies for up to `seconds`, stopping early once `done()` is true."""
        deadline = time.monotonic() + seconds
        while True:
            data = self._read_some()
            if data:
                for reply in self.parser.feed(data):
                    handle(reply)
                if done is not None and done():
                    return
            if time.monotonic() >= deadline:
                return

    def detect_echo(self, probe_id: int = 253, seconds: float = 0.02) -> bool:
        """Send a ping to an unused ID and check whether our own bytes come back."""
        self.echo = False
        self._skip.clear()
        self.drain()
        packet = make_packet(probe_id, INST_PING)
        self.port.write(packet)
        self.port.flush()
        got = bytearray()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and len(got) < len(packet):
            waiting = self.port.in_waiting
            got += self.port.read(waiting if waiting else 1)
        self.echo = bytes(got[:len(packet)]) == packet
        self.drain()
        return self.echo
