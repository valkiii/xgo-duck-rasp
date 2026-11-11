"""The ported firmware speaks exactly the binary ABI of upstream wire.py."""
import math

import wire  # upstream
from xgoduck_pi import mcu as m
from xgoduck_pi.sim import SimImu


def test_struct_sizes_match_upstream():
    assert m.STATE.size == wire.STATE.size == 240
    assert m.COMMAND.size == wire.COMMAND.size == 80
    assert m.CAL.size == wire.CAL.size == 38
    assert m.SERVO.size == wire.SERVO.size == 12
    assert m.RAW.size == wire.RAW.size == 66
    assert m.SERVO_REPLY.size == wire.SERVO_REPLY.size == 12


def test_state_packet_decodes_with_upstream():
    angles = [float(i) for i in range(15)]
    vels = [-float(i) for i in range(15)]
    targets = [0.5 * i for i in range(15)]
    pkt = m.STATE.pack(b'DQS1', 1, 0b10011, 0x7FFF, 7, 1000, 990, 6, 12000, 10000, 300, 2, 1, 0,
                       0, 0, -16384, 32, -64, 0, *angles, *vels, *targets)
    fb = wire.decode_state(pkt)
    assert fb.imu_ok and fb.enabled and fb.servo_debugging and not fb.calibrating
    assert fb.mask == 0x7FFF and fb.seq == 7 and fb.sample_us == 1000
    assert math.isclose(fb.acc[2], -9.8)
    assert fb.gyro == (1.0, -2.0, 0.0)
    assert list(fb.angles) == angles and list(fb.velocities) == vels and list(fb.targets) == targets


def test_raw_and_servo_reply_decode_with_upstream():
    raw = m.RAW.pack(b'DQR1', 0x00FF, *range(15), *range(100, 115))
    assert wire.decode_raw(raw) == {'mask': 0xFF, 'pos': list(range(15)), 'zero': list(range(100, 115))}
    reply = m.SERVO_REPLY.pack(b'DQS2', m.SERVO_READ, 12, 1, 0, 2047, 2005)
    assert wire.decode_servo_reply(reply) == {'op': m.SERVO_READ, 'target_id': 12, 'ok': True,
                                              'raw_pos': 2047, 'zero_pos': 2005}


def test_upstream_packets_are_accepted_and_bad_ones_counted():
    v = m.VirtualMCU(lambda: None, SimImu())
    v.notify('duck_command', wire.encode_command(5, 1234, wire.ARM, [1.0] * 15))
    assert v._latest.seq == 5 and v._latest.mode == m.ARM and v._latest.targets == (1.0,) * 15
    v.notify('duck_cal', wire.encode_cal(wire.CAL_FINISH_ONE, index=3))
    assert v._pending_cal[:2] == (m.CAL_FINISH_ONE, 3)
    v.notify('duck_servo', wire.encode_servo(wire.SERVO_SET_ID, 1, new_id=10))
    assert v._pending_servo[:3] == (m.SERVO_SET_ID, 1, 10)

    bad = bytearray(wire.encode_command(6, 1, wire.HOLD, [0.0] * 15))
    bad[0:4] = b'XXXX'
    v.notify('duck_command', bytes(bad))
    v.notify('duck_cal', wire.encode_cal(wire.CAL_FINISH_ONE, index=15))
    v.notify('duck_command', b'short')
    assert v.stats.invalid_commands == 3
    assert v._latest.seq == 5  # the bad command did not replace the good one


def test_wraparound_helpers():
    assert m.elapsed(5, 0xFFFFFFF0) == 21
    assert m.s32(0xFFFFFFFF) == -1
    assert m.s32(1 - 0xFFFFFFFF) == 2
