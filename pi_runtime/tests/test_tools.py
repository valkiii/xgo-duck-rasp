import itertools
import math
import sys
from pathlib import Path

from xgoduck_pi import scs_bus as scs
from xgoduck_pi.imu import axes_from_poses, parse_axes
from xgoduck_pi.sim import SimSerial, SimServo

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import bus_check  # noqa: E402


def test_orientation_solver_recovers_every_mounting():
    tilt = math.radians(35)
    count = 0
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            spec = ','.join(('+' if s > 0 else '-') + 'xyz'[i] for i, s in zip(perm, signs))
            try:
                axes = parse_axes(spec)
            except ValueError:
                continue

            def sensor(body):  # specific force, body frame -> sensor frame
                s = [0.0] * 3
                for row, (i, sign) in enumerate(axes):
                    s[i] = sign * body[row]
                return s
            up = sensor([0, 0, 1])
            nose_down = sensor([-math.sin(tilt), 0, math.cos(tilt)])
            assert axes_from_poses(up, nose_down) == spec
            count += 1
    assert count == 24


def test_bus_check_setup_and_scan(capsys):
    port = SimSerial(servos=[SimServo(1, 900)], echo=True)
    bus = scs.ScsBus(port)
    assert bus.detect_echo()
    assert bus_check.setup(bus, 1, 21) == 0
    servo = port.servos[0]
    assert servo.id == 21 and servo.goal == 2047 and (servo.perm_kp, servo.perm_kd) == (5, 20)
    assert not servo.torque
    assert bus_check.scan(bus) == [21]
    out = capsys.readouterr().out
    assert 'ID 1 -> 21' in out and 'missing assembly IDs' in out


def test_bus_check_refuses_duplicate_ids(capsys):
    port = SimSerial(servos=[SimServo(1), SimServo(10)])
    bus = scs.ScsBus(port)
    assert bus_check.setup(bus, 1, 10) == 1
    assert port.servos[0].id == 1
    assert 'already on the bus' in capsys.readouterr().out


# ---- imu_check --bias ----------------------------------------------------------
import imu_check  # noqa: E402


class _RestingImu:
    def __init__(self, gyro):
        self.gyro = gyro

    def read_sensor(self):
        return [0, 1, 0], list(self.gyro)


def test_measure_bias_averages_the_raw_gyro():
    mean, std = imu_check.measure_bias(_RestingImu((0.5, -1.25, 2.0)), seconds=0.05, interval=0.001)
    assert mean == [0.5, -1.25, 2.0] and std == [0.0, 0.0, 0.0]
    assert imu_check.bias_line(mean) == 'gyro_bias = [0.500, -1.250, 2.000]'


def test_write_bias_replaces_or_inserts_the_line(tmp_path):
    cfg = tmp_path / 'config.toml'
    cfg.write_text('[servo_bus]\nport = "auto"\n\n[imu]\ntype = "mpu6050"\ngyro_bias = [0.0, 0.0, 0.0]\n\n[battery]\nwarn_v = 7.0\n')
    imu_check.write_bias(cfg, [0.1, -0.2, 0.3])
    import tomllib
    data = tomllib.loads(cfg.read_text())
    assert data['imu']['gyro_bias'] == [0.1, -0.2, 0.3] and data['battery']['warn_v'] == 7.0
    cfg.write_text('[imu]\ntype = "mpu6050"\n\n[battery]\nwarn_v = 7.0\n')
    imu_check.write_bias(cfg, [1, 2, 3])
    data = tomllib.loads(cfg.read_text())
    assert data['imu']['gyro_bias'] == [1.0, 2.0, 3.0] and data['imu']['type'] == 'mpu6050'
    cfg.write_text('[battery]\nwarn_v = 7.0\n')
    import pytest
    with pytest.raises(ValueError):
        imu_check.write_bias(cfg, [0, 0, 0])


def test_shipped_config_parses_with_new_keys():
    import tomllib
    from pathlib import Path
    data = tomllib.loads((Path(__file__).resolve().parents[1] / 'config.toml').read_text())
    assert data['imu']['gyro_bias'] == [0.0, 0.0, 0.0]
    assert data['battery'] == {'warn_v': 7.0, 'cutoff_v': 6.6}
    assert data['http_host'] == '0.0.0.0'
