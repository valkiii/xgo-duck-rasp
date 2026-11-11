"""CAL_SET_ZEROS without a bus, battery/temperature monitoring, reply routing."""
import json
import threading
import time

import controller as controller_module
from conftest import wait_for
from test_robot import Heartbeat
from xgoduck_pi import mcu as m
from xgoduck_pi import scs_bus as scs
from xgoduck_pi import web
from xgoduck_pi.sim import SimImu, SimSerial


# ---- calibration zeros survive a closed / failing adapter ------------------
def test_zeros_pushed_while_the_adapter_is_closed_are_not_lost(tmp_path, monkeypatch):
    saved = [z + 13 for z in m.ZERO_POS]            # differs from the factory zeros
    zero_file = tmp_path / 'zero_pos.json'
    zero_file.write_text(json.dumps({'version': 1, 'zero_pos': saved}))
    monkeypatch.setattr(controller_module, 'ZERO_PATH', zero_file)
    adapter_present = threading.Event()
    ports = []

    def open_port():
        if not adapter_present.is_set():
            raise OSError('no such device')
        port = SimSerial(zero_pos=m.ZERO_POS, default_deg=m.DEFAULT_DEG)
        ports.append(port)
        return port

    mcu = m.VirtualMCU(open_port, SimImu(), echo='auto')
    ctl = controller_module.Controller(mcu)
    for name, fn in (('duck_state', ctl.receive), ('duck_boot_pd', ctl.receive_boot_pd),
                     ('duck_raw', ctl.receive_raw), ('duck_servo_reply', ctl.receive_servo_reply)):
        mcu.provide(name, fn)
    mcu.start()
    try:
        # The controller pushes its zeros on the first duck_raw, with the adapter still missing.
        assert wait_for(lambda: ctl._zeros_pushed, 3)
        assert mcu.bus is None
        assert wait_for(lambda: [mo.zero_pos for mo in mcu.motors] == saved, 2)
        time.sleep(0.5)                              # several duck_raw reports (5 Hz) adopt MCU zeros
        assert ctl.status()['zero_pos'] == saved
        assert mcu.bus is None
        # Adapter shows up later: reconnect must keep the file zeros, not reset to factory.
        adapter_present.set()
        assert wait_for(lambda: mcu.bus is not None, 3)
        time.sleep(0.5)
        assert [mo.zero_pos for mo in mcu.motors] == saved
        assert ctl.status()['zero_pos'] == saved
        assert json.loads(zero_file.read_text())['zero_pos'] == saved
    finally:
        mcu.stop()


def test_other_cal_ops_still_need_the_bus():
    mcu = m.VirtualMCU(lambda: (_ for _ in ()).throw(OSError('x')), SimImu())
    mcu._pending_cal = (m.CAL_ENTER, 0, 0, (0,) * 15)
    assert mcu.host_mode == m.HOST_NORMAL            # nothing applies it without a bus


# ---- reply routing ------------------------------------------------------------
def test_len4_replies_are_routed_by_pending_request():
    mcu = m.VirtualMCU(lambda: None, SimImu())
    reply = scs.Reply(10, 4, 0, bytes((79, 41)))
    mcu._handle_reply(reply)                          # nothing pending: ignored
    assert mcu.stats.pd_mask == 0 and not mcu._aux_seen
    mcu._boot_reading = True
    mcu._handle_reply(reply)                          # boot KP/KD sync read
    assert mcu.stats.pd_mask == 1 and (mcu.stats.boot_p[0], mcu.stats.boot_d[0]) == (79, 41)
    mcu._boot_reading = False
    mcu._aux_id = 10
    mcu._handle_reply(scs.Reply(11, 4, 0, bytes((1, 2))))   # other servo: not ours
    assert not mcu._aux_seen
    mcu._handle_reply(reply)
    assert mcu._aux_seen and mcu._aux_val == (79, 41) and mcu.stats.pd_mask == 1


# ---- battery and temperature -----------------------------------------------------
def test_voltage_and_temperature_are_read_slowly(robot):
    ctl, mcu, ports = robot
    bus = ports[-1]
    bus.servos[7].temp_c = 58
    assert wait_for(lambda: len(mcu.servo_temps) == 15 and mcu.bus_voltage() is not None, 3)
    time.sleep(2.0)
    reads = [p for p in bus.written if p[4] == scs.INST_READ and p[5] == scs.ADDR_VOLTAGE]
    assert 0 < len(reads) <= 15 * 4              # ~one sweep of 15 per second, nowhere near per tick
    p = web.pi_status(mcu)
    assert p['bus_voltage'] == 7.9 and not p['low_battery'] and not p['battery_warn']
    assert p['servo_temp_max'] == 58 and p['servo_temps']['22'] == 58 and len(p['servo_temps']) == 15
    assert mcu.stats.pd_mask == (1 << 15) - 1 and ctl.status()['feedback_hz'] > 50  # boot KP/KD still routed


def test_low_battery_refuses_arm_but_does_not_drop_a_walking_robot(robot, monkeypatch):
    ctl, mcu, ports = robot
    monkeypatch.setattr(m, 'BATTERY_SUSTAIN_S', 0.3)
    bus = ports[-1]
    with Heartbeat(ctl):
        ctl.set_mode('hold')
        assert wait_for(lambda: ctl.status().get('enabled'), 2)
        bus.set_voltage(6.8)                          # between cutoff 6.6 and warn 7.0
        assert wait_for(lambda: web.pi_status(mcu)['battery_warn'], 3)
        assert not mcu.battery_low
        bus.set_voltage(6.2)
        assert wait_for(lambda: mcu.battery_low, 4)
        assert web.pi_status(mcu)['low_battery']
        time.sleep(0.5)
        assert ctl.status()['enabled'] and all(sv.torque for sv in bus.servos)  # NOT forced limp
    assert wait_for(lambda: not ctl.status()['enabled'], 3)
    with Heartbeat(ctl):
        ctl.set_mode('hold')
        assert not wait_for(lambda: ctl.status().get('enabled'), 1.2)
        assert mcu.stats.arm_refused_battery > 0
        assert not any(sv.torque for sv in bus.servos)
        bus.set_voltage(7.8)
        assert wait_for(lambda: not mcu.battery_low, 3)
        # Upstream sends ARM once per mode change, so a refused arm needs a fresh mode selection.
        ctl.set_mode('shadow')
        ctl.set_mode('hold')
        assert wait_for(lambda: ctl.status().get('enabled'), 3)


def test_garbled_voltage_is_ignored():
    mcu = m.VirtualMCU(lambda: None, SimImu())
    mcu._record_aux(10, 0, 30)
    mcu._record_aux(11, 5, 0)
    assert mcu.bus_voltage() is None and mcu.servo_temps == {10: (30, mcu.servo_temps[10][1])}
    assert not mcu.battery_low
