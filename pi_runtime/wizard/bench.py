"""Servo bench: the URT-2 plugged into this computer, used for ID setup and bus checks."""
from __future__ import annotations

import glob
import random
import threading
import time

from xgoduck_pi import scs_bus as scs
from xgoduck_pi.mcu import MOTOR_IDS, ZERO_POS, DEFAULT_DEG
from xgoduck_pi.sim import SimSerial, SimServo

JOINTS = {
    10: 'left hip yaw', 11: 'left hip roll', 12: 'left hip pitch', 13: 'left knee', 14: 'left ankle',
    20: 'right hip yaw', 21: 'right hip roll', 22: 'right hip pitch', 23: 'right knee', 24: 'right ankle',
    30: 'neck pitch', 31: 'head pitch', 32: 'head yaw', 33: 'head roll', 34: 'mouth',
}
SIM_BENCH, SIM_ROBOT = 'sim:bench', 'sim:robot'
REPLY_S = 0.006


def list_ports() -> list[dict]:
    ports = []
    try:
        from serial.tools import list_ports as lp
        for p in lp.comports():
            usb_name = any(k in p.device for k in ('usbmodem', 'usbserial', 'wchusb', 'ttyACM', 'ttyUSB'))
            if p.vid is None and not usb_name:
                continue  # Bluetooth headsets, debug consoles, on-board UARTs
            if p.device.startswith('/dev/tty.'):
                continue  # macOS lists each port twice; /dev/cu.* is the one to open
            ports.append({'device': p.device, 'description': p.description if p.description != 'n/a' else '',
                          'likely': p.vid == 0x1A86 or usb_name})
    except Exception:
        for device in sorted(glob.glob('/dev/cu.usb*') + glob.glob('/dev/cu.wch*') + glob.glob('/dev/ttyACM*')):
            ports.append({'device': device, 'description': '', 'likely': True})
    ports.sort(key=lambda p: not p['likely'])
    ports.append({'device': SIM_BENCH, 'description': 'Simulator: one new servo at a time (no hardware)', 'likely': False})
    ports.append({'device': SIM_ROBOT, 'description': 'Simulator: assembled robot, 15 servos (no hardware)', 'likely': False})
    return ports


class BenchError(Exception):
    pass


class Bench:
    """One open servo bus shared by the wizard's requests (serialized by a lock)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.bus: scs.ScsBus | None = None
        self.device = ''
        self.sim: SimSerial | None = None

    # ---- connection ---------------------------------------------------------
    def status(self) -> dict:
        return {'connected': self.bus is not None, 'device': self.device,
                'echo': bool(self.bus and self.bus.echo), 'simulated': self.sim is not None}

    def connect(self, device: str, baud: int = 1_000_000) -> dict:
        with self.lock:
            self._close()
            if device == SIM_BENCH:
                self.sim = SimSerial(servos=[SimServo(1, random.randint(1200, 2900))], echo=True)
                port = self.sim
            elif device == SIM_ROBOT:
                self.sim = SimSerial(zero_pos=ZERO_POS, default_deg=DEFAULT_DEG)
                port = self.sim
            else:
                import serial
                try:
                    port = serial.Serial(device, baud, timeout=0, write_timeout=0.1)
                except (OSError, ValueError) as exc:
                    raise BenchError(f'cannot open {device}: {exc}') from exc
            self.bus = scs.ScsBus(port)
            self.bus.detect_echo()
            self.device = device
            return self.status()

    def disconnect(self) -> dict:
        with self.lock:
            self._close()
            return self.status()

    def _close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.set_torque(False)
                self.bus.port.close()
            except Exception:
                pass
        self.bus, self.device, self.sim = None, '', None

    def _need_bus(self) -> scs.ScsBus:
        if self.bus is None:
            raise BenchError('connect the URT-2 first')
        return self.bus

    def _guard(self, fn, *args):
        with self.lock:
            bus = self._need_bus()
            try:
                return fn(bus, *args)
            except OSError as exc:
                self._close()
                raise BenchError(f'lost the adapter ({exc}); reconnect it') from exc

    # ---- primitives -------------------------------------------------------
    @staticmethod
    def _request(bus, send, match, seconds=REPLY_S):
        found = []
        bus.drain()
        send()
        bus.flush()
        bus.collect(seconds, lambda r: found.append(r) if match(r) else None, lambda: bool(found))
        return found[0] if found else None

    def _ping(self, bus, servo_id):
        return self._request(bus, lambda: bus.ping(servo_id),
                             lambda r: r.servo_id == servo_id and r.length == 2) is not None

    def _read(self, bus, servo_id):
        r = self._request(bus, lambda: bus.read(servo_id, scs.ADDR_PRESENT, scs.PRESENT_LEN),
                          lambda r: r.servo_id == servo_id and r.length == 8 and r.error == 0)
        return None if r is None else scs.word(r.params[0], r.params[1])

    # ---- operations ---------------------------------------------------------
    def scan(self) -> dict:
        def run(bus):
            found = {}
            for servo_id in range(254):
                if self._ping(bus, servo_id):
                    found[servo_id] = self._read(bus, servo_id)
            return {'ids': sorted(found), 'positions': found,
                    'missing': [i for i in MOTOR_IDS if i not in found]}
        return self._guard(run)

    def read_all(self) -> dict:
        def run(bus):
            return {'positions': {i: self._read(bus, i) for i in MOTOR_IDS}}
        return self._guard(run)

    def setup(self, old_id: int, new_id: int, kp: int = 5, kd: int = 20) -> dict:
        """Upstream Servo setup page sequence: change ID, center at 2047, store KP/KD."""
        if not (0 <= old_id <= 253 and 0 <= new_id <= 253):
            raise BenchError('IDs are 0-253')

        def run(bus):
            log = []
            if not self._ping(bus, old_id):
                raise BenchError(f'no servo answers at ID {old_id}')
            if old_id != new_id:
                if self._ping(bus, new_id):
                    raise BenchError(f'ID {new_id} already answers: connect only the servo you are setting')
                bus.set_torque(False)
                bus.unlock(old_id)
                time.sleep(0.002)
                bus.set_id(old_id, new_id)
                time.sleep(0.05)
                if not self._ping(bus, new_id):
                    raise BenchError(f'ID write sent, but ID {new_id} does not answer; scan again')
                if self._ping(bus, old_id):
                    raise BenchError('both the old and new ID answer: connect one servo only')
                log.append(f'ID {old_id} → {new_id}')
            bus.set_run_kp_kd()
            bus.set_torque(True)
            bus.write_single_goal_raw(new_id, 2047)
            bus.flush()
            time.sleep(0.6)
            position = self._read(bus, new_id)
            log.append(f'centered: target 2047, reads {position}')
            bus.unlock(new_id)
            time.sleep(0.002)
            bus.set_perm_kp_kd(new_id, kp, kd)
            log.append(f'stored KP {kp} / KD {kd}')
            bus.set_torque(False)
            return {'ok': True, 'id': new_id, 'position': position, 'log': log}
        return self._guard(run)

    def wiggle(self, servo_id: int, steps: int = 60) -> dict:
        """Move one servo a few degrees and back, to see which joint it is. Torque only on that ID."""
        def run(bus):
            pos = self._read(bus, servo_id)
            if pos is None:
                raise BenchError(f'ID {servo_id} does not answer')
            bus.write_single_goal_raw(servo_id, pos)
            bus.write_byte(servo_id, scs.ADDR_TORQUE, 1)
            bus.flush()
            for target in (pos + steps, pos - steps, pos):
                bus.write_single_goal_raw(servo_id, target)
                bus.flush()
                time.sleep(0.35)
            bus.write_byte(servo_id, scs.ADDR_TORQUE, 0)
            bus.flush()
            return {'ok': True, 'id': servo_id, 'position': self._read(bus, servo_id)}
        return self._guard(run)

    def sim_new_servo(self) -> dict:
        """Simulator only: unplug the servo on the bench and plug in a fresh one (ID 1)."""
        with self.lock:
            if self.sim is None or self.device != SIM_BENCH:
                raise BenchError('only available with the bench simulator')
            self.sim.servos[:] = [SimServo(1, random.randint(1200, 2900))]
            return {'ok': True}
