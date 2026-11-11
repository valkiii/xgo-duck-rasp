#!/usr/bin/env python3
"""Check the IMU wiring and find the `axes` setting for config.toml.

    python tools/imu_check.py            # guided: robot upright, then nose-down
    python tools/imu_check.py --live     # print body-frame gravity with the configured axes
    python tools/imu_check.py --sample   # one averaged reading as JSON (used by the setup wizard)

Run it on the Pi with the IMU mounted in its final place on the robot.
"""
import argparse
import json
import sys
import time
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from xgoduck_pi.imu import make_imu, axes_from_poses  # noqa: E402


def average(imu, seconds=1.0):
    acc_sum, n = [0.0, 0.0, 0.0], 0
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        acc, _ = imu.read_sensor()
        acc_sum = [a + b for a, b in zip(acc_sum, acc)]
        n += 1
        time.sleep(0.01)
    return [a / n for a in acc_sum]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=str(ROOT / 'config.toml'))
    ap.add_argument('--live', action='store_true')
    ap.add_argument('--sample', action='store_true')
    args = ap.parse_args()
    cfg = tomllib.loads(Path(args.config).read_text()).get('imu', {})
    address = str(cfg.get('address', 'auto'))
    imu = make_imu(cfg.get('type', 'mpu6050'), bus_number=int(cfg.get('i2c_bus', 1)),
                   address=None if address == 'auto' else int(address, 0),
                   axes=cfg.get('axes', '-z,-x,+y'), dlpf=int(cfg.get('dlpf', 2)))
    if not imu.init():
        if args.sample:
            print(json.dumps({'error': f'IMU not found at 0x{imu.address:02x} on I2C bus {imu.bus_number}: '
                                       f'{imu.last_error or "no reply"}'}))
            return 1
        print(f'IMU not found on I2C bus {imu.bus_number} at 0x{imu.address:02x}: {imu.last_error}')
        print('Check wiring (VCC 3.3 V, GND, SDA pin 3, SCL pin 5, AD0 to 3.3 V) and run: i2cdetect -y 1')
        return 1
    if args.sample:
        acc = average(imu)
        imu.update()
        print(json.dumps({'type': type(imu).__name__, 'address': f'0x{imu.address:02x}', 'acc': acc,
                          'body_grav': [v / 16384 for v in imu.acc_raw], 'axes': cfg.get('axes', '-z,-x,+y')}))
        return 0
    print(f'{type(imu).__name__} found at 0x{imu.address:02x}')

    if args.live:
        print('Body frame: x forward, y left, z up. Upright should read grav ~ (0, 0, -1).')
        while True:
            imu.update()
            g = [v / 16384 for v in imu.acc_raw]
            w = [v / 32 for v in imu.gyro_raw]
            print(f'\rgrav ({g[0]:+.2f}, {g[1]:+.2f}, {g[2]:+.2f})  gyro deg/s ({w[0]:+6.1f}, {w[1]:+6.1f}, {w[2]:+6.1f})',
                  end='', flush=True)
            time.sleep(0.1)

    input('1) Hold the robot upright and still (as when standing), then press Enter...')
    up = average(imu)
    input('2) Tilt it nose-down by about 30-45 degrees, hold still, press Enter...')
    nose_down = average(imu)
    try:
        axes = axes_from_poses(up, nose_down)
    except ValueError as exc:
        print(f'Could not decide: {exc}')
        return 1
    print(f'\nSet this in config.toml under [imu]:\n    axes = "{axes}"')
    if axes != cfg.get('axes', '-z,-x,+y'):
        print(f'(currently "{cfg.get("axes")}")')
    else:
        print('That is already the configured value.')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print()
