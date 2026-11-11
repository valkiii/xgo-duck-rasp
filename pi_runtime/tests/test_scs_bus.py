from xgoduck_pi import scs_bus as scs
from xgoduck_pi.sim import SimSerial, SimServo, status


def test_packet_checksum():
    assert scs.make_packet(1, scs.INST_PING) == bytes.fromhex('ffff010201fb')
    assert scs.make_packet(0xFE, scs.INST_WRITE, bytes((40, 0))) == bytes.fromhex('fffffe04032800d2')


def test_parser_accepts_like_firmware():
    p = scs.ReplyParser()
    good = status(10, bytes((0x10, 0x08, 0x05, 0x80, 0, 0)))
    replies = p.feed(b'\x00\xff' + good[:5]) + p.feed(good[5:])  # junk + split delivery
    assert replies == [scs.Reply(10, 8, 0, bytes((0x10, 0x08, 0x05, 0x80, 0, 0)))]
    corrupt = bytearray(good)
    corrupt[-1] ^= 1
    assert p.feed(bytes(corrupt)) == []
    # LEN values the firmware does not accept are skipped
    assert p.feed(status(10, bytes(range(5)))) == []  # LEN 7


def test_word_and_speed_decoding():
    assert scs.word(0xFF, 0x0F) == 4095
    assert scs.word(0x00, 0x80) == -32768
    assert scs.speed_sign_magnitude(0x05, 0x80) == -5
    assert scs.speed_sign_magnitude(0x05, 0x00) == 5


def _ping(bus, servo_id):
    seen = []
    bus.drain()
    bus.ping(servo_id)
    bus.collect(0.01, lambda r: seen.append(r) if r.length == 2 and r.servo_id == servo_id else None,
                lambda: bool(seen))
    return bool(seen)


def test_echoing_adapter_is_detected_and_stripped():
    port = SimSerial(servos=[SimServo(1)], echo=True)
    bus = scs.ScsBus(port)
    assert bus.detect_echo()
    assert _ping(bus, 1)
    # An echoed ping looks like a valid 2-byte status; it must not count as a reply.
    assert not _ping(bus, 99)


def test_echo_would_fake_a_ping_if_not_stripped():
    bus = scs.ScsBus(SimSerial(servos=[SimServo(1)], echo=True))
    bus.echo = False
    assert _ping(bus, 99)  # why detect_echo() exists


def test_non_echoing_adapter():
    bus = scs.ScsBus(SimSerial(servos=[SimServo(1)]))
    assert not bus.detect_echo()
    assert _ping(bus, 1) and not _ping(bus, 2)


def test_sync_read_and_write_roundtrip():
    servos = [SimServo(i, 2000 + i) for i in (10, 11, 12)]
    port = SimSerial(servos=servos)
    bus = scs.ScsBus(port)
    got = {}
    bus.sync_read((10, 11, 12), scs.ADDR_PRESENT, scs.PRESENT_LEN)
    bus.collect(0.01, lambda r: got.__setitem__(r.servo_id, scs.word(r.params[0], r.params[1])))
    assert got == {10: 2010, 11: 2011, 12: 2012}
    bus.sync_write_goals((10, 11, 12), (100, 5000, -3))
    assert [s.goal for s in servos] == [100, 4095, 0]  # saturated like satPos()
