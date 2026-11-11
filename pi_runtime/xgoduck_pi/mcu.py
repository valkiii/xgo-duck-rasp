"""The UNO Q's STM32 firmware (sketch/sketch.ino), ported to a Linux thread.

On the UNO Q the host Python talks to an STM32 over Arduino Bridge: it sends
duck_command / duck_cal / duck_servo packets and receives duck_state (100 Hz),
duck_raw, duck_servo_reply and duck_boot_pd. VirtualMCU keeps that exact binary
interface (Bridge.provide / bridge.notify), so the upstream controller, policies
and recovery logic run unchanged on the Pi. Underneath, it drives the servo bus
and the IMU directly with the firmware's loop, timeouts and safety rules.
"""
from __future__ import annotations

import logging
import math
import os
import struct
import threading
import time
from dataclasses import dataclass, field

from . import scs_bus as scs

log = logging.getLogger('xgoduck.mcu')

# protocol.h
TICK_US = 10_000
COMMAND_TTL_US = 250_000
MAX_SOURCE_AGE_US = 150_000
DISARM, ARM, POLICY, HOLD = range(4)
HOST_NORMAL, HOST_CAL, HOST_SERVO_DEBUG = range(3)
CAL_ENTER, CAL_EXIT, CAL_SET_RAW, CAL_FINISH_ONE, CAL_SET_ZEROS = range(5)
(SERVO_ENTER, SERVO_EXIT, SERVO_UNLOCK, SERVO_SET_ID, SERVO_GOTO,
 SERVO_SET_PERM_KP_KD, SERVO_READ, SERVO_PING) = range(8)

COMMAND = struct.Struct('<4sIIIBBH15f')
STATE = struct.Struct('<4sBBH10I6h45f')
CAL = struct.Struct('<4sBBH15H')
SERVO = struct.Struct('<4sBBBBBBH')
RAW = struct.Struct('<4sH15H15H')
SERVO_REPLY = struct.Struct('<4sBBBBHH')

# duck_config.h
NUM_MOTORS = 15
SERVO_REPLY_US = 6000
AUX_REPLY_US = 1500           # extra wait for one voltage/temperature read (reply is ~100 us)
AUX_SWEEP_S = 1.0             # a sweep over all servos starts about once a second
VOLT_VALID_RAW = (30, 300)    # 3.0 V .. 30 V in 0.1 V units; anything else is a garbled read
VOLT_MAX_AGE_S = 5.0
BATTERY_SUSTAIN_S = 3.0
BATTERY_HYSTERESIS_V = 0.1
M_N, M_A = 4095.0, 360.0
SERVO_SPEED_UNIT = 50.0
CAL_CENTER_POS = 2047
TEMP_KP_CAL, TEMP_KD_CAL = 3, 0
MOTOR_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
ZERO_POS = (2236, 2005, 1912, 1866, 1945, 2127, 2135, 1914, 2228, 2206, 2550, 2060, 2128, 2034, 2190)
MOTOR_SIGN = (-1.0,) * NUM_MOTORS
DEFAULT_DEG = (0.0, -5.0, -23.0, 0.0, 23.0, 0.0, 5.0, 23.0, 0.0, -23.0, 20.0, 20.0, 0.0, 0.0, 0.0)

U32 = 0xFFFFFFFF


def micros() -> int:
    return (time.monotonic_ns() // 1000) & U32


def elapsed(now: int, before: int) -> int:
    return (now - before) & U32


def s32(x: int) -> int:
    x &= U32
    return x - 0x1_0000_0000 if x & 0x8000_0000 else x


@dataclass
class Motor:
    id: int
    zero_pos: int
    sign: float
    des_angle_deg: float
    fb_pos: int = 0
    des_pos: int = CAL_CENTER_POS
    fb_angle_deg: float = 0.0
    fb_vel_deg: float = 0.0


@dataclass
class Command:
    seq: int
    source_us: int
    ttl_us: int
    mode: int
    targets: tuple


@dataclass
class Stats:
    overruns: int = 0
    report_drops: int = 0
    invalid_commands: int = 0
    bus_errors: int = 0
    bus_error: str = ''
    echo: bool = False
    arm_refused_battery: int = 0
    pd_mask: int = 0
    boot_p: list = field(default_factory=lambda: [0] * NUM_MOTORS)
    boot_d: list = field(default_factory=lambda: [0] * NUM_MOTORS)


class VirtualMCU:
    """Drop-in for the Arduino Bridge object the upstream Controller expects."""

    def __init__(self, open_port, imu, *, echo='auto', realtime=False,
                 battery_warn_v=7.0, battery_cutoff_v=6.6) -> None:
        # open_port() -> a pyserial-like object; called again after a USB disconnect.
        self._open_port = open_port
        self.imu = imu
        self.echo_setting = echo
        self.realtime = realtime
        self.bus: scs.ScsBus | None = None
        self.motors = [Motor(i, z, s, d, fb_pos=z) for i, z, s, d in
                       zip(MOTOR_IDS, ZERO_POS, MOTOR_SIGN, DEFAULT_DEG)]
        self._index = {m.id: n for n, m in enumerate(self.motors)}
        self._callbacks = {}
        self.stats = Stats()
        # Firmware mutexes: commandMu / hostMu (from the host side), motorsMu, stateMu.
        self._command_mu = threading.Lock()
        self._host_mu = threading.Lock()
        self._motors_mu = threading.Lock()
        self._state_mu = threading.Lock()
        self._report_ready = threading.Condition(self._state_mu)
        self._latest: Command | None = None
        self._pending_cal = None
        self._pending_servo = None
        self._servo_reply = None
        self._snapshot = None
        self._report_pending = False
        self.host_mode = HOST_NORMAL
        self.raw_notify_every_tick = False
        self._rx_mask = 0
        self._ping_id = None
        self._ping_seen = False
        self._read_id = None
        self._read_seen = False
        self._read_pos = 0
        # Battery / temperature monitor: one small read per tick, see _poll_aux().
        self.battery_warn_v = battery_warn_v
        self.battery_cutoff_v = battery_cutoff_v
        self.battery_low = False
        self.servo_volts = {}     # id -> (volts, monotonic time)
        self.servo_temps = {}     # id -> (deg C, monotonic time)
        self._low_since = None
        self._aux_id = None
        self._aux_seen = False
        self._aux_val = (0, 0)
        self._aux_next = NUM_MOTORS
        self._aux_sweep_at = 0.0
        self._boot_reading = False
        self._last_arm_warn = 0.0
        self._stop = threading.Event()
        self._threads = []

    # ---- Bridge interface ----------------------------------------------
    def provide(self, name: str, fn) -> None:
        self._callbacks[name] = fn

    def notify(self, name: str, payload: bytes) -> None:
        if name == 'duck_command':
            self._receive_command(payload)
        elif name == 'duck_cal':
            self._receive_cal(payload)
        elif name == 'duck_servo':
            self._receive_servo(payload)
        else:
            raise ValueError(f'unknown bridge method {name!r}')

    def _invalid(self) -> None:
        with self._command_mu:
            self.stats.invalid_commands += 1

    def _receive_command(self, data) -> None:
        cmd = None
        if isinstance(data, (bytes, bytearray)) and len(data) == COMMAND.size:
            x = COMMAND.unpack(data)
            targets = x[7:]
            if (x[0] == b'DQC1' and x[4] <= HOLD and not x[5] and not x[6]
                    and 0 < x[3] <= COMMAND_TTL_US and all(math.isfinite(v) for v in targets)):
                cmd = Command(seq=x[1], source_us=x[2], ttl_us=x[3], mode=x[4], targets=targets)
        with self._command_mu:
            if cmd is None:
                self.stats.invalid_commands += 1
            else:
                self._latest = cmd

    def _receive_cal(self, data) -> None:
        pkt = None
        if isinstance(data, (bytes, bytearray)) and len(data) == CAL.size:
            x = CAL.unpack(data)
            magic, op, index, mask, raw = x[0], x[1], x[2], x[3], x[4:]
            valid = magic == b'DQL1' and op <= CAL_SET_ZEROS
            if op == CAL_FINISH_ONE:
                valid = valid and index < NUM_MOTORS
            if op in (CAL_SET_RAW, CAL_SET_ZEROS):
                valid = valid and all(raw[i] <= scs.POS_MAX for i in range(NUM_MOTORS) if mask & (1 << i))
            if valid:
                pkt = (op, index, mask, raw)
        if pkt is None:
            self._invalid()
            return
        with self._host_mu:
            self._pending_cal = pkt

    def _receive_servo(self, data) -> None:
        pkt = None
        if isinstance(data, (bytes, bytearray)) and len(data) == SERVO.size:
            magic, op, target, new_id, kp, kd, _res, raw_pos = SERVO.unpack(data)
            valid = magic == b'DQV1' and op <= SERVO_PING and target <= 253
            if op == SERVO_GOTO:
                valid = valid and raw_pos <= scs.POS_MAX
            if op == SERVO_SET_ID:
                valid = valid and new_id <= 253
            if valid:
                pkt = (op, target, new_id, kp, kd, raw_pos)
        if pkt is None:
            self._invalid()
            return
        with self._host_mu:
            self._pending_servo = pkt

    # ---- lifecycle -----------------------------------------------------
    def start(self) -> None:
        for name, target in (('duck_control100', self._control_task), ('duck_report100', self._report_task)):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        with self._report_ready:
            self._report_ready.notify_all()
        for t in self._threads:
            t.join(timeout)
        # Leave the robot limp, as the firmware does whenever it is not enabled.
        if self.bus is not None:
            try:
                for _ in range(3):
                    self.bus.set_torque(False)
                    time.sleep(0.005)
            except Exception:  # port already gone
                pass

    # ---- servo feedback --------------------------------------------------
    def _handle_reply(self, r: scs.Reply) -> None:
        """Dispatch of parseServoRx() once a packet's checksum is good."""
        if r.length == 2 and self._ping_id is not None and r.servo_id == self._ping_id:
            self._ping_seen = True  # even an error status proves the address answered
            return
        idx = self._index.get(r.servo_id, -1)
        if r.length == 4:
            # Two-byte replies come from exactly two requests: the pending voltage/temperature
            # read, or the one-off boot KP/KD sync read. Anything else is stale and ignored.
            if r.error == 0 and self._aux_id is not None and r.servo_id == self._aux_id:
                self._aux_val = (r.params[0], r.params[1])
                self._aux_seen = True
            elif r.error == 0 and self._boot_reading and idx >= 0:
                self.stats.boot_p[idx], self.stats.boot_d[idx] = r.params[0], r.params[1]
                self.stats.pd_mask |= 1 << idx
            return
        if r.length != 8 or r.error != 0:
            return
        pos = scs.word(r.params[0], r.params[1])
        if self._read_id is not None and r.servo_id == self._read_id:
            self._read_seen = True
            self._read_pos = pos & 0xFFFF
        if idx < 0:
            return
        vel = scs.speed_sign_magnitude(r.params[2], r.params[3])
        self._rx_mask |= 1 << idx
        m = self.motors[idx]
        with self._motors_mu:
            m.fb_pos = pos & 0xFFFF
            m.fb_angle_deg = (pos - m.zero_pos) * M_A / M_N * m.sign
            m.fb_vel_deg = vel * SERVO_SPEED_UNIT * (360.0 / 4096.0) * m.sign

    def _send_des_pos(self) -> None:
        for m in self.motors:
            m.des_pos = scs.sat_pos(m.zero_pos + int(m.des_angle_deg / M_A * M_N * m.sign))
        self.bus.sync_write_goals(MOTOR_IDS, (m.des_pos for m in self.motors))

    def _send_raw_pos(self) -> None:
        self.bus.sync_write_goals(MOTOR_IDS, (m.des_pos for m in self.motors))

    def _wait_replies(self, done=None) -> None:
        self.bus.collect(SERVO_REPLY_US / 1e6, self._handle_reply, done)

    # ---- host operations ----------------------------------------------
    def _apply_cal(self, pkt) -> None:
        op, index, mask, raw = pkt
        bus = self.bus
        if op == CAL_ENTER:
            self.host_mode = HOST_CAL
            self.raw_notify_every_tick = True
            with self._motors_mu:
                for m in self.motors:
                    m.des_pos = CAL_CENTER_POS
            bus.set_temp_kp_kd(TEMP_KP_CAL, TEMP_KD_CAL)
            bus.set_torque(True)
            with self._motors_mu:
                self._send_raw_pos()
            bus.flush()
        elif op == CAL_EXIT:
            self.host_mode = HOST_NORMAL
            self.raw_notify_every_tick = False
            bus.set_torque(False)
            bus.set_run_kp_kd()
        elif op == CAL_SET_RAW:
            if self.host_mode != HOST_CAL:
                return
            with self._motors_mu:
                for i, m in enumerate(self.motors):
                    if mask & (1 << i):
                        m.des_pos = scs.sat_pos(raw[i])
        elif op == CAL_FINISH_ONE:
            if self.host_mode != HOST_CAL or index >= NUM_MOTORS:
                return
            with self._motors_mu:
                self.motors[index].zero_pos = self.motors[index].fb_pos
        elif op == CAL_SET_ZEROS:
            with self._motors_mu:
                for i, m in enumerate(self.motors):
                    if mask & (1 << i):
                        m.zero_pos = scs.sat_pos(raw[i])

    def _apply_servo(self, pkt) -> None:
        op, target, new_id, kp, kd, raw_pos = pkt
        bus = self.bus
        ok, reply_pos, reply_zero = 1, 0, 0
        debug = self.host_mode == HOST_SERVO_DEBUG
        if op == SERVO_ENTER:
            self.host_mode = HOST_SERVO_DEBUG
            self.raw_notify_every_tick = True
            bus.set_torque(False)
            bus.set_run_kp_kd()
        elif op == SERVO_EXIT:
            self.host_mode = HOST_NORMAL
            self.raw_notify_every_tick = False
            bus.set_torque(False)
            bus.set_run_kp_kd()
        elif not debug:
            ok = 0
        elif op == SERVO_UNLOCK:
            bus.unlock(target)
        elif op == SERVO_SET_ID:
            bus.set_torque(False)
            bus.unlock(target)
            time.sleep(0.002)
            bus.set_id(target, new_id)
        elif op == SERVO_GOTO:
            bus.set_run_kp_kd()
            bus.set_torque(True)
            bus.write_single_goal_raw(target, raw_pos)
            bus.flush()
        elif op == SERVO_SET_PERM_KP_KD:
            bus.unlock(target)
            time.sleep(0.002)
            bus.set_perm_kp_kd(target, kp, kd)
        elif op == SERVO_PING:
            bus.drain()
            self._ping_id, self._ping_seen = target, False
            bus.ping(target)
            bus.flush()
            self._wait_replies(lambda: self._ping_seen)
            ok = 1 if self._ping_seen else 0
            self._ping_id = None
        elif op == SERVO_READ:
            bus.drain()
            self._read_id, self._read_seen, self._read_pos = target, False, 0
            bus.read(target, scs.ADDR_PRESENT, scs.PRESENT_LEN)
            bus.flush()
            self._wait_replies(lambda: self._read_seen)
            ok, reply_pos = (1 if self._read_seen else 0), self._read_pos
            self._read_id = None
            idx = self._index.get(target, -1)
            if idx >= 0:
                with self._motors_mu:
                    reply_zero = self.motors[idx].zero_pos
        reply = SERVO_REPLY.pack(b'DQS2', op, target, ok, 0, reply_pos & 0xFFFF, reply_zero & 0xFFFF)
        with self._host_mu:
            self._servo_reply = reply

    # ---- bus connection --------------------------------------------------
    def _connect(self) -> bool:
        try:
            port = self._open_port()
        except Exception as exc:
            self._bus_failed(exc)
            return False
        bus = scs.ScsBus(port)
        if self.echo_setting == 'auto':
            bus.detect_echo()
        else:
            bus.echo = bool(self.echo_setting)
        self.stats.echo = bus.echo
        self.bus = bus
        self.stats.bus_error = ''
        log.info('servo bus open (echo %s)', 'on' if bus.echo else 'off')
        for _ in range(3):
            bus.set_torque(False)
            time.sleep(0.005)
        bus.set_run_kp_kd()
        # Read temporary KP/KD once (address 50, 2 bytes); missing IDs stay unset.
        bus.drain()
        bus.sync_read(MOTOR_IDS, scs.ADDR_TEMP_KP, 2)
        bus.flush()
        self._boot_reading = True
        try:
            bus.collect(0.020, self._handle_reply)
        finally:
            self._boot_reading = False
        return True

    # ---- battery voltage and servo temperature ----------------------------
    def bus_voltage(self):
        # Median of the servo voltage readings from the last few seconds, or None.
        now = time.monotonic()
        vals = sorted(v for v, t in list(self.servo_volts.values()) if now - t <= VOLT_MAX_AGE_S)
        return vals[len(vals) // 2] if vals else None

    def servo_temp_max(self):
        now = time.monotonic()
        vals = [v for v, t in list(self.servo_temps.values()) if now - t <= 10 * VOLT_MAX_AGE_S]
        return max(vals) if vals else None

    def _poll_aux(self) -> None:
        # At most one extra 2-byte read per tick: sweeps all servos about once a second.
        now = time.monotonic()
        if self._aux_next >= NUM_MOTORS:
            if now < self._aux_sweep_at:
                return
            self._aux_next, self._aux_sweep_at = 0, now + AUX_SWEEP_S
        servo_id = MOTOR_IDS[self._aux_next]
        self._aux_next += 1
        bus = self.bus
        bus.drain()
        self._aux_id, self._aux_seen = servo_id, False
        bus.read(servo_id, scs.ADDR_VOLTAGE, scs.VOLT_TEMP_LEN)
        bus.flush()
        bus.collect(AUX_REPLY_US / 1e6, self._handle_reply, lambda: self._aux_seen)
        self._aux_id = None
        if self._aux_seen:
            self._record_aux(servo_id, *self._aux_val)

    def _record_aux(self, servo_id: int, volt_raw: int, temp_raw: int) -> None:
        now = time.monotonic()
        if VOLT_VALID_RAW[0] <= volt_raw <= VOLT_VALID_RAW[1]:
            self.servo_volts[servo_id] = (volt_raw / 10.0, now)
        if 0 < temp_raw <= 150:
            self.servo_temps[servo_id] = (temp_raw, now)
        v = self.bus_voltage()
        if v is None:
            return
        if v < self.battery_cutoff_v:
            if self._low_since is None:
                self._low_since = now
            elif now - self._low_since >= BATTERY_SUSTAIN_S and not self.battery_low:
                self.battery_low = True
                log.warning('battery %.1f V below cutoff %.1f V: new ARM commands are refused',
                            v, self.battery_cutoff_v)
        elif v >= self.battery_cutoff_v + BATTERY_HYSTERESIS_V:
            self._low_since = None
            if self.battery_low:
                self.battery_low = False
                log.info('battery back to %.1f V: ARM allowed again', v)

    def _bus_failed(self, exc) -> None:
        self.stats.bus_errors += 1
        message = f'servo bus: {exc}'
        if message != self.stats.bus_error:
            log.warning(message)
        self.stats.bus_error = message
        if self.bus is not None:
            try:
                self.bus.port.close()
            except Exception:
                pass
        self.bus = None

    # ---- the 100 Hz loop ---------------------------------------------------
    def _set_realtime(self) -> None:
        if not self.realtime or not hasattr(os, 'sched_setscheduler'):
            return
        try:
            os.sched_setscheduler(threading.get_native_id(), os.SCHED_FIFO, os.sched_param(20))
            log.info('control thread running SCHED_FIFO')
        except (PermissionError, OSError) as exc:
            log.info('real-time scheduling unavailable (%s); running normal priority', exc)

    def _control_task(self) -> None:
        self._set_realtime()
        self._connect()
        self.imu.init()
        deadline = previous = micros()
        seq = accepted_seq = last_command_us = last_imu_us = last_healthy_us = 0
        ttl = COMMAND_TTL_US
        enabled = timed_out = have_command = previous_enabled = False
        next_reconnect = 0.0
        while not self._stop.is_set():
            start = micros()
            period = elapsed(start, previous)
            previous = start

            if self.bus is None and time.monotonic() >= next_reconnect:
                next_reconnect = time.monotonic() + 1.0
                self._connect()
            bus = self.bus

            with self._host_mu:
                cal, self._pending_cal = self._pending_cal, None
                servo, self._pending_servo = self._pending_servo, None
            with self._command_mu:
                cmd, self._latest = self._latest, None
                bad = self.stats.invalid_commands

            try:
                if cal is not None and (bus is not None or cal[0] == CAL_SET_ZEROS):
                    # CAL_SET_ZEROS only updates memory, so it must not be dropped while the
                    # adapter is closed: the controller pushes it once, then adopts our zeros.
                    self._apply_cal(cal)
                if bus is not None:
                    if servo is not None:
                        self._apply_servo(servo)
                elif servo is not None:
                    with self._host_mu:
                        self._servo_reply = SERVO_REPLY.pack(b'DQS2', servo[0], servo[1], 0, 0, 0, 0)

                bus_taken = self.host_mode != HOST_NORMAL
                if cmd is not None and not bus_taken:
                    fresh = cmd.mode == DISARM or elapsed(start, cmd.source_us) <= MAX_SOURCE_AGE_US
                    ordered = not have_command or cmd.mode == DISARM or s32(cmd.seq - accepted_seq) > 0
                    if fresh and ordered:
                        accepted_seq, have_command, last_command_us, ttl = cmd.seq, True, start, cmd.ttl_us
                        if cmd.mode == DISARM:
                            enabled = timed_out = False
                        if cmd.mode == ARM and not enabled and self.battery_low:
                            self.stats.arm_refused_battery += 1
                            if time.monotonic() - self._last_arm_warn > 5.0:
                                self._last_arm_warn = time.monotonic()
                                log.warning('ARM refused: battery %s V is below the %.1f V cutoff',
                                            self.bus_voltage(), self.battery_cutoff_v)
                        elif (cmd.mode == ARM and not enabled and self.imu.ok and bus is not None
                                and elapsed(start, last_imu_us) < 50_000):
                            enabled, timed_out, last_healthy_us = True, False, start
                            bus.set_run_kp_kd()
                        with self._motors_mu:
                            for m, target in zip(self.motors, cmd.targets):
                                m.des_angle_deg = target
                    else:
                        self._invalid()
                if bus_taken:
                    enabled = timed_out = False
                elif enabled and (elapsed(start, last_command_us) > ttl
                                  or elapsed(start, last_healthy_us) > 100_000):
                    enabled, timed_out = False, True

                self._rx_mask = 0
                if bus is not None:
                    if self.host_mode == HOST_CAL:
                        with self._motors_mu:
                            self._send_raw_pos()
                        bus.flush()
                    elif enabled:
                        with self._motors_mu:
                            self._send_des_pos()
                        bus.flush()
                    if not bus_taken:
                        if enabled != previous_enabled or not enabled:
                            bus.set_torque(enabled)
                        previous_enabled = enabled
                    else:
                        previous_enabled = False
                    bus.drain()
                    bus.sync_read(MOTOR_IDS, scs.ADDR_PRESENT, scs.PRESENT_LEN)
                    bus.flush()
                if self.imu.update():
                    last_imu_us = micros()
                if bus is not None:
                    full = (1 << NUM_MOTORS) - 1
                    self._wait_replies(lambda: self._rx_mask == full)
                    if self._rx_mask == full:  # bus idle again; room for one small extra read
                        self._poll_aux()
            except OSError as exc:  # serial.SerialException subclasses OSError: USB unplugged etc.
                self._bus_failed(exc)
                enabled = previous_enabled = False
                self._rx_mask = 0
            except Exception:
                log.exception('control tick failed')
                enabled = False
            if self.imu.ok:
                last_healthy_us = micros()
            if self.bus is None:
                enabled = False

            rx_mask = self._rx_mask
            flags = ((1 if self.imu.ok else 0) | (2 if enabled else 0) | (4 if timed_out else 0)
                     | (8 if self.host_mode == HOST_CAL else 0) | (16 if self.host_mode == HOST_SERVO_DEBUG else 0))
            seq = (seq + 1) & U32
            with self._motors_mu:
                angles = [m.fb_angle_deg for m in self.motors]
                vels = [m.fb_vel_deg for m in self.motors]
                targets = [m.des_angle_deg for m in self.motors]
                raw = RAW.pack(b'DQR1', rx_mask, *[m.fb_pos for m in self.motors],
                               *[m.zero_pos for m in self.motors])
            work = elapsed(micros(), start)
            with self._state_mu:
                if self._report_pending:
                    self.stats.report_drops += 1
                state = STATE.pack(
                    b'DQS1', 1, flags, rx_mask, seq, start, last_imu_us, accepted_seq,
                    elapsed(start, last_command_us) if have_command else U32,
                    period, work, self.stats.overruns, self.stats.report_drops, bad,
                    *self.imu.acc_raw, *self.imu.gyro_raw, *angles, *vels, *targets)
                self._snapshot = (state, raw)
                self._report_pending = True
                self._report_ready.notify()

            deadline = (deadline + TICK_US) & U32
            now = micros()
            if s32(deadline - now) <= 0:
                missed = elapsed(now, deadline) // TICK_US + 1
                self.stats.overruns += missed
                deadline = (deadline + missed * TICK_US) & U32
            remaining = s32(deadline - micros())
            if remaining > 0:
                self._stop.wait(remaining / 1e6)

    def _report_task(self) -> None:
        reports = 0
        call = self._callbacks.get
        while not self._stop.is_set():
            with self._report_ready:
                while not self._report_pending and not self._stop.is_set():
                    self._report_ready.wait(0.5)
                if self._stop.is_set():
                    return
                state, raw = self._snapshot
                self._report_pending = False
            try:
                if fn := call('duck_state'):
                    fn(state)
                if (self.raw_notify_every_tick or reports % 20 == 0) and (fn := call('duck_raw')):
                    fn(raw)
                with self._host_mu:
                    reply, self._servo_reply = self._servo_reply, None
                if reply is not None and (fn := call('duck_servo_reply')):
                    fn(reply)
                reports += 1
                if reports % 100 == 0 and (fn := call('duck_boot_pd')):
                    st = self.stats
                    fn(bytes((st.pd_mask & 0xFF, st.pd_mask >> 8, *st.boot_p, *st.boot_d)))
            except Exception:
                log.exception('report callback failed')
