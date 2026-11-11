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


# ---- brownout detection and gyro bias, with a fake SMBus ------------------------
class FakeSMBus:
    """Just enough MPU6050: a register file, and sample registers 0x3B..0x48."""

    def __init__(self):
        self.regs = {0x75: 0x68, 0x6B: 0x01}
        self.sample = [0, 16384, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # words: ax ay az temp gx gy gz
        self.closed = False
        self.fail_io = False

    def write_byte_data(self, addr, reg, value):
        if self.fail_io:
            raise OSError('i2c')
        self.regs[reg] = 0x40 if (reg == 0x6B and value == 0x80) else value

    def read_i2c_block_data(self, addr, reg, n):
        if self.fail_io:
            raise OSError('i2c')
        if reg == 0x3B:
            out = []
            for w in self.sample[:7]:
                out += [(w >> 8) & 0xFF, w & 0xFF]
            return out
        return [self.regs.get(reg + i, 0) for i in range(n)]

    def close(self):
        self.closed = True


def make_mpu(**kw):
    imu = MPU6050(**kw)
    bus = FakeSMBus()
    imu._bus = bus
    imu.health_interval = 0.0          # check PWR_MGMT_1 on every sample
    assert imu.init()
    return imu, bus


def test_sleep_bit_marks_the_imu_lost_and_reinit_recovers():
    imu, bus = make_mpu()
    assert imu.update() and imu.ok
    bus.regs[0x6B] = 0x40               # power dip: chip reset into sleep, reads still "work"
    bus.sample = [0] * 7
    assert not imu.update()
    assert not imu.ok and not imu.found and 'asleep' in imu.last_error
    bus.sample = [0, 16384, 0, 0, 0, 0, 0] + [0] * 0
    imu._last_probe = 0.0               # the 1 s re-probe is due
    imu.update()
    assert imu.found and imu.ok and bus.regs[0x6B] == 0x01
    assert imu.update()


def test_twenty_low_g_samples_mark_the_imu_lost():
    imu, bus = make_mpu()
    imu.health_interval = 1e9           # isolate the accelerometer check
    bus.sample = [0, 1000, 0, 0, 0, 0, 0]   # 0.06 g
    for _ in range(19):
        assert imu.update()
    assert not imu.update() and not imu.ok and not imu.found
    assert '0.06 g' in imu.last_error


def test_a_few_low_g_samples_in_free_fall_are_fine():
    imu, bus = make_mpu()
    imu.health_interval = 1e9
    for _ in range(3):
        bus.sample = [0, 0, 0, 0, 0, 0, 0]
        assert imu.update()
        bus.sample = [0, 16384, 0, 0, 0, 0, 0]
        assert imu.update()
    for _ in range(40):                 # the counter resets on every normal sample
        bus.sample = [0, 0, 0, 0, 0, 0, 0] if _ % 10 else [0, 16384, 0, 0, 0, 0, 0]
        assert imu.update()


def test_gyro_bias_is_subtracted_in_the_sensor_frame_before_axis_mapping():
    # Sensor gyro (x, y, z) reads (1.0, 2.0, -3.0) deg/s at rest; bias equals that.
    lsb = MPU6050.GYRO_LSB_PER_DPS
    raw = [round(1.0 * lsb), round(2.0 * lsb), round(-3.0 * lsb)]

    def sample(bias):
        imu, bus = make_mpu(axes='-z,-x,+y', gyro_bias=bias)
        bus.sample = [0, 16384, 0, 0, *raw]
        assert imu.update()
        return imu.gyro_raw
    zero = sample((1.0, 2.0, -3.0))
    assert all(abs(v) <= 1 for v in zero)
    plain = sample((0.0, 0.0, 0.0))
    # body = (-sz, -sx, +sy) in 32 LSB per deg/s
    assert plain == pytest.approx([96, -32, 64], abs=1)
    # a bias on sensor x only moves body y (= -sx)
    only_x = sample((1.0, 0.0, 0.0))
    assert only_x[1] == pytest.approx(0, abs=1) and only_x[0] == plain[0] and only_x[2] == plain[2]


def test_bad_gyro_bias_is_rejected():
    with pytest.raises(ValueError):
        MPU6050(gyro_bias=(1.0, 2.0))
