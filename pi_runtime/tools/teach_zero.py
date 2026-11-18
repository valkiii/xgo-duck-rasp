#!/usr/bin/env python3
"""Teach the joint zeros by hand, with torque off, instead of the Calibration page.

    python tools/teach_zero.py                       # Pi at 192.168.2.87
    python tools/teach_zero.py --host 192.168.2.87 --ssh valkiii_zero@192.168.2.87

Runs on the Mac. The robot software must be in shadow/off mode (torque off),
which is how it boots. Pose the robot in the zero pose (calibration/zero_pose.png:
legs straight down, feet flat, neck vertical, head level, beak closed), press
Enter, and the averaged encoder readings become upstream/data/zero_pos.json on
the Pi. The service is then restarted so the controller loads and pushes them.
Nothing moves: the Calibration page's CAL_ENTER would drive every servo to 2047.
"""
import argparse
import json
import select
import subprocess
import sys
import time

IDS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
GROUPS = [('left leg ', IDS[0:5]), ('right leg', IDS[5:10]), ('neck/head', IDS[10:15])]
WRAP_MARGIN = 300  # raw counts; a zero this close to 0/4095 leaves little room before the wrap
REMOTE_PATH = '~/xgo-duck/upstream/data/zero_pos.json'


def status(host):
    # System curl, not urllib: macOS Local Network privacy blocks Homebrew Python.
    out = subprocess.run(['curl', '-s', '-m', '3', f'http://{host}:9527/api/status'],
                         capture_output=True, text=True)
    if out.returncode:
        raise OSError(f'curl exit {out.returncode}')
    return json.loads(out.stdout)


def line(raw):
    by_id = dict(zip(IDS, raw))
    return '   '.join(name + ' ' + ' '.join(f'{i}:{by_id[i]:4d}' for i in ids) for name, ids in GROUPS)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='192.168.2.87')
    ap.add_argument('--ssh', default='valkiii_zero@192.168.2.87')
    ap.add_argument('--seconds', type=float, default=1.5, help='averaging window after Enter')
    args = ap.parse_args()

    s = status(args.host)
    if s.get('mode') not in ('shadow', 'off') or s.get('enabled'):
        sys.exit(f"mode is {s.get('mode')!r} (enabled={s.get('enabled')}); disarm / leave calibrate first")
    if s.get('missing_joint_ids'):
        sys.exit(f"servos missing on the bus: {s['missing_joint_ids']}")

    print('Torque is off. Pose the zero pose by hand, then press Enter (Ctrl+C aborts).\n')
    while True:
        raw = status(args.host)['raw_pos']
        sys.stdout.write('\r' + line(raw))
        sys.stdout.flush()
        if select.select([sys.stdin], [], [], 0.2)[0]:
            sys.stdin.readline()
            break

    print(f'\nHold still, averaging {args.seconds:.1f} s...')
    samples, end = [], time.monotonic() + args.seconds
    while time.monotonic() < end:
        samples.append(status(args.host)['raw_pos'])
        time.sleep(0.05)
    zero = [round(sum(col) / len(col)) for col in zip(*samples)]
    spread = [max(col) - min(col) for col in zip(*samples)]

    print(line(zero))
    moved = [i for i, sp in zip(IDS, spread) if sp > 15]
    if moved:
        print(f'WARNING: moved while averaging: {moved}')
    near_wrap = [i for i, z in zip(IDS, zero) if z < WRAP_MARGIN or z > 4095 - WRAP_MARGIN]
    if near_wrap:
        print(f'WARNING: zero near the 0/4095 wrap for {near_wrap}: the horn may need re-seating one spline')

    if input('\nSave these as the zeros and restart the robot service? [y/N] ').strip().lower() != 'y':
        sys.exit('not saved')
    payload = json.dumps({'version': 1, 'ids': IDS, 'zero_pos': zero,
                          'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'source': 'teach_zero'}, indent=2)
    remote = (f'f={REMOTE_PATH}; [ -s "$f" ] && cp "$f" "$f.$(date +%Y%m%d-%H%M%S).bak"; '
              f'cat > "$f" && sudo systemctl restart xgo-duck')
    subprocess.run(['ssh', args.ssh, remote], input=payload, text=True, check=True)

    for _ in range(30):
        time.sleep(1)
        try:
            got = status(args.host).get('zero_pos')
        except OSError:
            continue
        if got == zero:
            print('Saved and loaded: the controller now reports these zeros.')
            return
    sys.exit('saved, but the controller did not report the new zeros within 30 s; check the service')


if __name__ == '__main__':
    main()
