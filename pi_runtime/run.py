#!/usr/bin/env python3
"""XGO Duck on a Raspberry Pi: upstream controller + ported firmware loop + web UI.

    python run.py                 # real hardware, settings from config.toml
    python run.py --sim           # simulated servos and IMU (any computer)
    python run.py --port /dev/cu.usbmodem1101 --no-imu   # e.g. servo setup from a Mac
"""
from __future__ import annotations

import argparse
import glob
import logging
import signal
import sys
import threading
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib

HERE = Path(__file__).resolve().parent
log = logging.getLogger('xgoduck')

PORT_GLOBS = ('/dev/serial/by-id/*CH34*', '/dev/serial/by-id/*1a86*', '/dev/ttyACM*', '/dev/ttyUSB*',
              '/dev/cu.wchusbserial*', '/dev/cu.usbmodem*', '/dev/cu.usbserial*')


def find_port(configured: str) -> str:
    if configured and configured != 'auto':
        return configured
    for pattern in PORT_GLOBS:
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    raise OSError('no USB serial adapter found (is the URT-2 plugged in?)')


class NoImu:
    """For servo setup without the IMU wired: always reports IMU missing."""
    ok = False
    acc_raw = [0, 0, 0]
    gyro_raw = [0, 0, 0]
    last_error = 'IMU disabled (--no-imu)'

    def init(self):
        return False

    def update(self):
        return False

    def close(self):
        pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=str(HERE / 'config.toml'))
    ap.add_argument('--sim', action='store_true', help='simulated servos and IMU')
    ap.add_argument('--port', help='serial port of the servo adapter (overrides config)')
    ap.add_argument('--no-imu', action='store_true', help='run without an IMU (servo setup only)')
    ap.add_argument('--http-port', type=int, help='web UI port (overrides config)')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format='%(asctime)s %(levelname)s %(name)s: %(message)s')

    cfg = tomllib.loads(Path(args.config).read_text())
    upstream = (HERE / cfg.get('upstream', 'upstream')).resolve()
    if not (upstream / 'python' / 'controller.py').is_file():
        log.error('upstream runtime not found at %s; run ./fetch_upstream.sh first', upstream)
        return 1
    sys.path.insert(0, str(upstream / 'python'))
    sys.setswitchinterval(0.001)  # as upstream: bound GIL handoff between the threads

    from controller import Controller  # upstream, unchanged
    from xgoduck_pi.mcu import VirtualMCU, ZERO_POS, DEFAULT_DEG
    from xgoduck_pi import web

    bus_cfg, imu_cfg, bat_cfg = cfg.get('servo_bus', {}), cfg.get('imu', {}), cfg.get('battery', {})
    if args.sim:
        from xgoduck_pi.sim import SimSerial, SimImu
        open_port = lambda: SimSerial(zero_pos=ZERO_POS, default_deg=DEFAULT_DEG)  # noqa: E731
        imu = SimImu()
        label = 'SIMULATED'
    else:
        import serial
        port_name = args.port or bus_cfg.get('port', 'auto')
        baud = int(bus_cfg.get('baud', 1_000_000))

        def open_port():
            name = find_port(port_name)
            log.info('opening %s at %d baud', name, baud)
            return serial.Serial(name, baud, timeout=0, write_timeout=0.1)
        if args.no_imu:
            imu = NoImu()
        else:
            from xgoduck_pi.imu import make_imu
            address = str(imu_cfg.get('address', 'auto'))
            imu = make_imu(imu_cfg.get('type', 'mpu6050'), bus_number=int(imu_cfg.get('i2c_bus', 1)),
                           address=None if address == 'auto' else int(address, 0),
                           axes=imu_cfg.get('axes', '-z,-x,+y'), dlpf=int(imu_cfg.get('dlpf', 2)),
                           gyro_bias=imu_cfg.get('gyro_bias', (0.0, 0.0, 0.0)))
        label = 'RASPBERRY PI'

    mcu = VirtualMCU(open_port, imu, echo=bus_cfg.get('echo', 'auto'),
                     realtime=bool(cfg.get('realtime', True)),
                     battery_warn_v=float(bat_cfg.get('warn_v', 7.0)),
                     battery_cutoff_v=float(bat_cfg.get('cutoff_v', 6.6)))
    log.info('loading policies (takes a few seconds on a Pi Zero)')
    controller = Controller(mcu)
    log.info('%s', controller.model_message)
    mcu.provide('duck_state', controller.receive)
    mcu.provide('duck_boot_pd', controller.receive_boot_pd)
    mcu.provide('duck_raw', controller.receive_raw)
    mcu.provide('duck_servo_reply', controller.receive_servo_reply)
    mcu.start()

    worker = threading.Thread(target=controller.run, name='policy50', daemon=True)
    worker.start()

    http_port = args.http_port or int(cfg.get('http_port', 9527))
    http_host = str(cfg.get('http_host', '0.0.0.0'))
    server = web.Server((http_host, http_port), web.make_routes(controller), upstream / 'assets',
                        mcu=mcu, label=label)
    web.serve(server)
    log.info('web UI on http://%s:%d/', http_host, http_port)

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        stop.wait()
    finally:
        log.info('stopping: torque off')
        controller.stop()
        worker.join(timeout=2)
        server.shutdown()
        mcu.stop()
        imu.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
