import random

import pytest

from xgoduck_pi.imu import parse_axes, to_body, MPU6050
from proc import SignalFilter  # upstream receive-side filter
import wire


def firmware_qmi(ax, ay, az, gx, gy, gz):
    """The mapping hard-coded in sketch/qmi8658.h updateImu()."""
    return [az, ax, -ay], [-gz, -gx, gy]


def test_default_axes_reproduce_firmware_qmi_mapping():
    axes = parse_axes('-z,-x,+y')
    for _ in range(200):
        a = [random.randint(-20000, 20000) for _ in range(3)]
        g = [random.randint(-20000, 20000) for _ in range(3)]
        assert to_body(axes, a, g) == firmware_qmi(*a, *g)


def test_mirror_mappings_are_rejected():
    with pytest.raises(ValueError):
        parse_axes('+x,+y,-z')
    with pytest.raises(ValueError):
        parse_axes('+x,+x,+z')
    assert parse_axes('+x,+y,+z') == ((0, 1), (1, 1), (2, 1))


def _gravity(acc_fw, gyro_fw=(0, 0, 0)):
    """Push one firmware sample through upstream decoding and the receive filter."""
    acc = [v * 9.8 / 16384 for v in acc_fw]
    gyro = [v / 32 for v in gyro_fw]
    f = SignalFilter()
    return f.update(acc, gyro, [0.0] * 15, [0.0] * 15, timestamp=1.0)


def test_upright_mpu_gives_policy_gravity_down():
    axes = parse_axes('-z,-x,+y')
    # Board on the back, Y arrow up: the accelerometer reads +1 g along sensor Y.
    acc_fw, _ = to_body(axes, (0, 16384, 0), (0, 0, 0))
    out = _gravity(acc_fw)
    assert out['grav'] == pytest.approx([0, 0, -1], abs=1e-6)


def test_nose_down_tilt_shows_positive_x():
    axes = parse_axes('-z,-x,+y')
    # Robot pitched 30 deg nose-down: sensor Z (pointing backwards) now has an upward component.
    import math
    s, c = math.sin(math.radians(30)), math.cos(math.radians(30))
    acc_fw, _ = to_body(axes, (0, 16384 * c, 16384 * s), (0, 0, 0))
    grav = _gravity(acc_fw)['grav']
    assert grav[0] == pytest.approx(s, abs=1e-3) and grav[2] == pytest.approx(-c, abs=1e-3)


def test_mpu_gyro_rescaled_to_qmi_units():
    axes = parse_axes('+x,+y,+z')
    _, gyro = to_body(axes, (0, 0, 0), (328, 0, -328), 32.0 / MPU6050.GYRO_LSB_PER_DPS)
    assert gyro == [320, 0, -320]  # 10 deg/s at 32 LSB per deg/s
    assert wire.STATE  # upstream decodes gyro as raw / 32
