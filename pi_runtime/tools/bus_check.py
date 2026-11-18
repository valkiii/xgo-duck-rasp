#!/usr/bin/env python3
"""Talk to the servo bus directly, without the full runtime. Works on the Pi and on a Mac.

    python tools/bus_check.py scan                 # find every servo ID (0-253) and its position
    python tools/bus_check.py setup 1 10           # one servo on the bus: ID 1 -> 10, center, gains
    python tools/bus_check.py hold [IDS] [--nudge 45]  # torque on where it stands (service stopped)
    python tools/bus_check.py zero [IDS] [--frac 0.5]  # ramp to the saved zeros and back (service stopped)
    python tools/bus_check.py --port /dev/cu.usbmodem1101 scan

`setup` does what the upstream Servo setup page does: change the ID, move to raw
2047 (center) and store KP 5 / KD 20. Connect ONE servo at a time.
The assembly IDs are 10-14 and 20-24 (legs), 30-33 (neck/head), 34 (mouth).
Servos need power (6-8.4 V on the adapter's power terminal) to answer.
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from xgoduck_pi import scs_bus as scs  # noqa: E402
from run import find_port  # noqa: E402


def open_bus(port_name, baud):
    import serial
    name = find_port(port_name or 'auto')
    bus = scs.ScsBus(serial.Serial(name, baud, timeout=0, write_timeout=0.1))
    echo = bus.detect_echo()
    print(f'{name} at {baud} baud, adapter echo {"on" if echo else "off"}')
    return bus


def request(bus, send, match, seconds=0.006):
    found = []
    bus.drain()
    send()
    bus.flush()
    bus.collect(seconds, lambda r: found.append(r) if match(r) else None, lambda: bool(found))
    return found[0] if found else None


def ping(bus, servo_id):
    return request(bus, lambda: bus.ping(servo_id), lambda r: r.servo_id == servo_id and r.length == 2)


def read_pos(bus, servo_id):
    r = request(bus, lambda: bus.read(servo_id, scs.ADDR_PRESENT, scs.PRESENT_LEN),
                lambda r: r.servo_id == servo_id and r.length == 8 and r.error == 0)
    return None if r is None else scs.word(r.params[0], r.params[1])


def scan(bus):
    found = []
    for servo_id in range(254):
        if ping(bus, servo_id):
            found.append(servo_id)
            print(f'  ID {servo_id:3d}  position {read_pos(bus, servo_id)}')
    print(f'{len(found)} servo(s) found' + ('' if found else ': check power, cable and baud rate'))
    expected = set(range(10, 15)) | set(range(20, 25)) | set(range(30, 35))
    if found and set(found) <= expected:
        missing = sorted(expected - set(found))
        if missing:
            print(f'missing assembly IDs: {missing}')
    return found


def setup(bus, old_id, new_id, kp=5, kd=20):
    if not ping(bus, old_id):
        print(f'no servo answers at ID {old_id}')
        return 1
    if old_id != new_id:
        if ping(bus, new_id):
            print(f'ID {new_id} is already on the bus; connect only the servo you are setting')
            return 1
        bus.set_torque(False)
        bus.unlock(old_id)
        time.sleep(0.002)
        bus.set_id(old_id, new_id)
        time.sleep(0.05)
        if not ping(bus, new_id) or ping(bus, old_id):
            print('ID write did not verify; check that only one servo is connected and try again')
            return 1
        print(f'ID {old_id} -> {new_id}')
    bus.set_run_kp_kd()
    bus.set_torque(True)
    bus.write_single_goal_raw(new_id, 2047)
    bus.flush()
    time.sleep(0.6)
    bus.unlock(new_id)
    time.sleep(0.002)
    bus.set_perm_kp_kd(new_id, kp, kd)
    print(f'centered at raw 2047 (reads {read_pos(bus, new_id)}), stored KP {kp} / KD {kd}')
    bus.set_torque(False)
    print(f'Done. Label this servo "{new_id}".')
    return 0


ALL_IDS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]


def hold(bus, ids, seconds, kp, kd, nudge):
    """Torque on where each servo already is (goal = present first), so nothing jumps."""
    start = {}
    for servo_id in ids:
        pos = read_pos(bus, servo_id)
        if pos is None:
            print(f'ID {servo_id} does not answer; nothing was enabled')
            return 1
        start[servo_id] = pos
    print('holding at ' + '  '.join(f'{i}:{p}' for i, p in start.items()) + f'  (KP {kp}, KD {kd})')
    bus.set_temp_kp_kd(kp, kd)
    bus.sync_write_goals(list(start), list(start.values()))
    bus.flush()
    for servo_id in ids:
        bus.write_byte(servo_id, scs.ADDR_TORQUE, 1)
    bus.flush()
    bus.set_temp_kp_kd(kp, kd)

    def report(goals):
        print('  ' + '  '.join(f'{i}:{(read_pos(bus, i) or 0) - goals[i]:+4d}' for i in ids) + '  (counts off target)')

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        report(start)
        time.sleep(0.5)
    if nudge:
        for servo_id in ids:
            print(f'ID {servo_id}: raw {nudge:+d} and back (about {nudge * 360 / 4096:+.0f} deg)')
            for step in list(range(1, 11)) + list(range(9, -1, -1)):
                bus.write_single_goal_raw(servo_id, start[servo_id] + nudge * step // 10)
                bus.flush()
                time.sleep(0.08)
            time.sleep(0.3)
        report(start)
    return 0


ZERO_FILE = ROOT / 'upstream' / 'data' / 'zero_pos.json'


def to_zero(bus, ids, frac, seconds, hold_s, kp, kd):
    """Ramp from the current pose to frac x the saved zeros, hold, ramp back, torque off."""
    import json
    zeros = dict(zip(ALL_IDS, json.loads(ZERO_FILE.read_text())['zero_pos']))
    start = {}
    for servo_id in ids:
        pos = read_pos(bus, servo_id)
        if pos is None:
            print(f'ID {servo_id} does not answer; nothing was enabled')
            return 1
        start[servo_id] = pos
    target = {i: round(start[i] + frac * (zeros[i] - start[i])) for i in ids}
    print('  '.join(f'{i}: {start[i]}->{target[i]} ({(target[i] - start[i]) * 360 / 4096:+.0f} deg)' for i in ids))
    bus.set_temp_kp_kd(kp, kd)
    bus.sync_write_goals(list(start), list(start.values()))
    bus.flush()
    for servo_id in ids:
        bus.write_byte(servo_id, scs.ADDR_TORQUE, 1)
    bus.flush()
    bus.set_temp_kp_kd(kp, kd)

    def ramp(a, b):
        steps = max(1, int(seconds / 0.04))
        for k in range(1, steps + 1):
            bus.sync_write_goals(ids, [round(a[i] + (b[i] - a[i]) * k / steps) for i in ids])
            bus.flush()
            time.sleep(0.04)

    ramp(start, target)
    end = time.monotonic() + hold_s
    while time.monotonic() < end:
        time.sleep(0.5)
        print('  at target, off by ' + '  '.join(f'{i}:{(read_pos(bus, i) or 0) - target[i]:+d}' for i in ids))
    ramp(target, start)
    time.sleep(0.3)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='serial port (default: first USB serial adapter)')
    ap.add_argument('--baud', type=int, default=1_000_000)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('scan')
    s = sub.add_parser('setup')
    s.add_argument('old_id', type=int)
    s.add_argument('new_id', type=int)
    h = sub.add_parser('hold', help='torque on at the current pose (stop the xgo-duck service first)')
    h.add_argument('ids', type=int, nargs='*', help='default: all 15')
    h.add_argument('--seconds', type=float, default=5)
    h.add_argument('--kp', type=int, default=4)
    h.add_argument('--kd', type=int, default=20)
    h.add_argument('--nudge', type=int, default=0, help='raw counts to move each servo and back (45 = 4 deg)')
    z = sub.add_parser('zero', help='ramp to the saved zeros, hold, ramp back (stop the service first)')
    z.add_argument('ids', type=int, nargs='*', help='default: both legs (10-14, 20-24)')
    z.add_argument('--frac', type=float, default=1.0, help='fraction of the way to the zeros (0-1)')
    z.add_argument('--seconds', type=float, default=4, help='ramp time each way')
    z.add_argument('--hold', type=float, default=5)
    z.add_argument('--kp', type=int, default=4)
    z.add_argument('--kd', type=int, default=20)
    args = ap.parse_args()
    bus = open_bus(args.port, args.baud)
    try:
        if args.cmd == 'scan':
            return 0 if scan(bus) else 1
        if args.cmd == 'hold':
            if abs(args.nudge) > 150:
                print('nudge is limited to 150 counts (13 deg)')
                return 1
            return hold(bus, args.ids or ALL_IDS, args.seconds, args.kp, args.kd, args.nudge)
        if args.cmd == 'zero':
            if not 0 <= args.frac <= 1:
                print('--frac is 0-1')
                return 1
            return to_zero(bus, args.ids or ALL_IDS[:10], args.frac, args.seconds, args.hold, args.kp, args.kd)
        if not (0 <= args.new_id <= 253):
            print('IDs are 0-253')
            return 1
        return setup(bus, args.old_id, args.new_id)
    finally:
        try:
            bus.set_torque(False)
        except Exception:
            pass
        bus.port.close()


if __name__ == '__main__':
    sys.exit(main())
