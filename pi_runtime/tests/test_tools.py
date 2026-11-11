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
