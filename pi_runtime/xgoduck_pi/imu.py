"""IMU drivers that reproduce the firmware's ImuSample (sketch/qmi8658.h).

The firmware hands the host int16 values in fixed units and axes:
  acc_raw  : gravity direction in the body frame, 16384 LSB per g
             (standing upright reads about (0, 0, -16384))
  gyro_raw : body angular rate, 32 LSB per deg/s
Body frame: x forward, y left, z up.

`axes` says which sensor axis lies along body x, y and z, e.g. "-z,-x,+y" means
body x = -sensor z, body y = -sensor x, body z = +sensor y. That is how the QMI8658
sits on the Luwu board (vertical on the robot's back, chip facing backwards), and the
default for an MPU6050 mounted the same way: flat against the back, components facing
out, silkscreen Y arrow pointing up, X arrow pointing to the robot's right.
Run tools/imu_check.py once to confirm or find the right string.
"""
from __future__ import annotations

import time
from typing import Optional, Sequence

INT16_MIN, INT16_MAX = -32768, 32767
QMI_GYRO_LSB_PER_DPS = 32.0
ACC_LSB_PER_G = 16384.0


def _clamp16(v: float) -> int:
    v = int(round(v))
    return INT16_MIN if v < INT16_MIN else INT16_MAX if v > INT16_MAX else v


def parse_axes(spec: str) -> tuple[tuple[int, int], ...]:
    """'-z,-x,+y' -> ((2,-1),(0,-1),(1,1)); rejects mirror images."""
    parts = [p.strip().lower() for p in spec.split(',')]
    if len(parts) != 3:
        raise ValueError(f'axes needs three entries, got {spec!r}')
    out = []
    for p in parts:
        sign = -1 if p.startswith('-') else 1
        name = p.lstrip('+-')
        if name not in ('x', 'y', 'z'):
            raise ValueError(f'bad axis {p!r} in {spec!r}')
        out.append(('xyz'.index(name), sign))
    if sorted(i for i, _ in out) != [0, 1, 2]:
        raise ValueError(f'axes must use x, y and z once each: {spec!r}')
    m = [[0] * 3 for _ in range(3)]
    for row, (col, sign) in enumerate(out):
        m[row][col] = sign
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
           - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    if det != 1:
        raise ValueError(f'axes {spec!r} is a mirror image, not a rotation; flip one sign')
    return tuple(out)


def to_body(axes, acc_sensor: Sequence[float], gyro_sensor: Sequence[float],
            gyro_scale: float = 1.0) -> tuple[list[int], list[int]]:
    """Sensor-frame specific force / rate -> firmware gravity-direction / rate in body axes."""
    acc = [_clamp16(-sign * acc_sensor[i]) for i, sign in axes]
    gyro = [_clamp16(sign * gyro_sensor[i] * gyro_scale) for i, sign in axes]
    return acc, gyro


class ImuLost(Exception):
    """The sensor answers on I2C but is no longer running (e.g. reset by a power dip)."""


class _I2CImu:
    """Common retry behaviour: probe again once a second while missing (as the firmware)."""

    def __init__(self, bus_number: int, address: int, axes: str) -> None:
        self.bus_number = bus_number
        self.address = address
        self.axes = parse_axes(axes)
        self.ok = False
        self.acc_raw = [0, 0, 0]
        self.gyro_raw = [0, 0, 0]
        self.found = False
        self._bus = None
        self._last_probe = 0.0
        self.last_error = ''

    def _open(self):
        if self._bus is None:
            from smbus2 import SMBus
            self._bus = SMBus(self.bus_number)
        return self._bus

    def _read(self, reg: int, length: int) -> bytes:
        return bytes(self._open().read_i2c_block_data(self.address, reg, length))

    def _write(self, reg: int, value: int) -> None:
        self._open().write_byte_data(self.address, reg, value)

    def init(self) -> bool:
        try:
            self.found = self._init()
            self.last_error = '' if self.found else 'IMU did not identify'
        except OSError as exc:
            self.found = False
            self.last_error = str(exc)
        self.ok = self.found
        return self.found

    def update(self) -> bool:
        if not self.found:
            now = time.monotonic()
            if now - self._last_probe >= 1.0:
                self._last_probe = now
                self.init()
            return False
        try:
            acc, gyro = self._sample()
        except ImuLost as exc:
            self.found = self.ok = False
            self.last_error = str(exc)
            self._last_probe = time.monotonic()  # re-init after the usual 1 s
            return False
        except OSError as exc:
            self.ok = False
            self.last_error = str(exc)
            return False
        self.acc_raw, self.gyro_raw = acc, gyro
        self.ok = True
        return True

    def close(self) -> None:
        if self._bus is not None:
            self._bus.close()
            self._bus = None


class MPU6050(_I2CImu):
    """InvenSense MPU-6050 / MPU-6500 family, e.g. GY-521 boards.

    AD0 tied high gives address 0x69, which avoids the PiSugar 3 RTC at 0x68.
    """
    WHO_AM_I_VALUES = {0x68, 0x69, 0x70, 0x71, 0x72, 0x73, 0x98}
    GYRO_LSB_PER_DPS = 32.8  # +-1000 deg/s range

    PWR_MGMT_1 = 0x6B
    SLEEP_BIT = 0x40
    MIN_ACC_G = 0.3          # |a| this low for LOW_ACC_SAMPLES in a row = sensor not measuring
    LOW_ACC_SAMPLES = 20

    def __init__(self, bus_number: int = 1, address: int = 0x69, axes: str = '-z,-x,+y',
                 dlpf: int = 2, gyro_bias=(0.0, 0.0, 0.0)) -> None:
        super().__init__(bus_number, address, axes)
        self.dlpf = dlpf
        self.who_am_i = None
        bias = [float(v) for v in gyro_bias]
        if len(bias) != 3:
            raise ValueError(f'gyro_bias needs three values (deg/s), got {gyro_bias!r}')
        self.gyro_bias = bias
        self.health_interval = 1.0
        self._last_health = 0.0
        self._low_acc = 0

    def _init(self) -> bool:
        self._low_acc = 0
        self._last_health = time.monotonic()
        who = self._read(0x75, 1)[0]
        self.who_am_i = who
        if who not in self.WHO_AM_I_VALUES:
            return False
        self._write(0x6B, 0x80)          # PWR_MGMT_1: device reset
        time.sleep(0.1)
        self._write(0x6B, 0x01)          # wake, clock from gyro X PLL
        time.sleep(0.01)
        self._write(0x19, 0x00)          # SMPLRT_DIV: 1 kHz internal rate
        self._write(0x1A, self.dlpf & 7)  # CONFIG: digital low-pass filter
        self._write(0x1B, 0x10)          # GYRO_CONFIG: +-1000 deg/s
        self._write(0x1C, 0x00)          # ACCEL_CONFIG: +-2 g (16384 LSB/g, same as the QMI setup)
        time.sleep(0.05)
        cfg = self._read(0x1A, 3)
        return cfg[0] & 7 == self.dlpf & 7 and cfg[1] & 0x18 == 0x10 and cfg[2] & 0x18 == 0x00

    def read_sensor(self):
        """Unrotated sensor-frame (acc in g, gyro in deg/s)."""
        acc_s, gyro_s = self._raw()
        return [v / ACC_LSB_PER_G for v in acc_s], [v / self.GYRO_LSB_PER_DPS for v in gyro_s]

    def _raw(self):
        d = self._read(0x3B, 14)

        def s16(i):
            v = (d[i] << 8) | d[i + 1]
            return v - 0x10000 if v & 0x8000 else v
        return (s16(0), s16(2), s16(4)), (s16(8), s16(10), s16(12))

    def _check_health(self, acc_s) -> None:
        # After a power dip the chip resets into sleep and returns zeros without I2C errors.
        mag = sum(v * v for v in acc_s) ** 0.5 / ACC_LSB_PER_G
        self._low_acc = self._low_acc + 1 if mag < self.MIN_ACC_G else 0
        if self._low_acc >= self.LOW_ACC_SAMPLES:
            raise ImuLost(f'MPU6050 accelerometer reads {mag:.2f} g for {self._low_acc} samples (reset?)')
        now = time.monotonic()
        if now - self._last_health >= self.health_interval:
            self._last_health = now
            if self._read(self.PWR_MGMT_1, 1)[0] & self.SLEEP_BIT:
                raise ImuLost('MPU6050 is asleep (PWR_MGMT_1 sleep bit set: power dip?)')

    def _sample(self):
        acc_s, gyro_s = self._raw()
        self._check_health(acc_s)
        gyro_s = [g - b * self.GYRO_LSB_PER_DPS for g, b in zip(gyro_s, self.gyro_bias)]
        return to_body(self.axes, acc_s, gyro_s, QMI_GYRO_LSB_PER_DPS / self.GYRO_LSB_PER_DPS)


class QMI8658(_I2CImu):
    """The QMI8658A used on the Luwu driver board, configured exactly like the firmware."""

    def __init__(self, bus_number: int = 1, address: Optional[int] = None, axes: str = '-z,-x,+y') -> None:
        super().__init__(bus_number, address or 0x6A, axes)
        self._candidates = (address,) if address else (0x6A, 0x6B)

    def _init(self) -> bool:
        for addr in self._candidates:
            self.address = addr
            try:
                if self._read(0x00, 1)[0] == 0x05:
                    break
            except OSError:
                continue
        else:
            return False
        time.sleep(0.01)
        self._write(0x60, 0xB0)  # soft reset
        time.sleep(0.02)
        if self._read(0x4D, 1)[0] != 0x80:
            return False
        for reg, value in ((0x08, 0x00), (0x02, 0x40), (0x03, 0x04), (0x04, 0x64), (0x06, 0x00), (0x08, 0x03)):
            self._write(reg, value)
        time.sleep(0.25)
        c = self._read(0x02, 7)
        return c[0] == 0x40 and c[1] == 0x04 and c[2] == 0x64 and c[4] == 0x00 and c[6] == 0x03

    def read_sensor(self):
        acc_s, gyro_s = self._raw()
        return [v / ACC_LSB_PER_G for v in acc_s], [v / QMI_GYRO_LSB_PER_DPS for v in gyro_s]

    def _raw(self):
        d = self._read(0x35, 12)

        def s16(i):
            v = d[i] | (d[i + 1] << 8)
            return v - 0x10000 if v & 0x8000 else v
        return (s16(0), s16(2), s16(4)), (s16(6), s16(8), s16(10))

    def _sample(self):
        acc_s, gyro_s = self._raw()
        return to_body(self.axes, acc_s, gyro_s)


def axes_from_poses(up, nose_down) -> str:
    """Mounting string from two averaged accelerometer readings (sensor frame, in g):
    `up` with the robot standing upright, `nose_down` with it pitched forward ~30-45 deg."""
    def unit(v):
        n = sum(x * x for x in v) ** 0.5
        return [x / n for x in v]
    up = unit(up)
    zi = max(range(3), key=lambda i: abs(up[i]))
    zs = 1 if up[zi] > 0 else -1          # specific force points up when standing
    nd = unit(nose_down)
    # Pitching nose-down tips the measured "up" towards body -x.
    rest = [nd[i] if i != zi else 0.0 for i in range(3)]
    xi = max(range(3), key=lambda i: abs(rest[i]))
    if abs(rest[xi]) < 0.25:
        raise ValueError('tilt the robot further nose-down (at least ~20 degrees)')
    xs = -1 if rest[xi] > 0 else 1
    x = [0, 0, 0]
    z = [0, 0, 0]
    x[xi], z[zi] = xs, zs
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    yi = max(range(3), key=lambda i: abs(y[i]))
    name = lambda i, s: ('+' if s > 0 else '-') + 'xyz'[i]  # noqa: E731
    return ','.join((name(xi, xs), name(yi, y[yi]), name(zi, zs)))


def make_imu(kind: str, **kwargs):
    kind = kind.lower()
    if kind == 'mpu6050':
        kwargs['address'] = kwargs.get('address') or 0x69
        return MPU6050(**kwargs)
    if kind == 'qmi8658':
        kwargs.pop('dlpf', None)
        kwargs.pop('gyro_bias', None)  # only the MPU6050 driver applies a bias
        return QMI8658(**kwargs)
    raise ValueError(f'unknown imu type {kind!r} (use mpu6050 or qmi8658)')
