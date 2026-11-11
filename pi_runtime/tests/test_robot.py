"""End to end: upstream Controller -> VirtualMCU -> simulated bus, as on the robot."""
import json
import math
import threading
import time
import urllib.request

import controller as controller_module
from conftest import wait_for
from xgoduck_pi import mcu as m
from xgoduck_pi.sim import SimSerial, SimServo, SimImu


class Heartbeat:
    """What the web page does: a command every 50 ms."""

    def __init__(self, ctl, twist=(0.0, 0.0, 0.0)):
        self.ctl, self.twist, self.stop = ctl, list(twist), threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self.stop.is_set():
            self.ctl.command(twist=self.twist)
            self.stop.wait(0.05)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.thread.join()


def expected_goal(n):
    return m.ZERO_POS[n] + int(m.DEFAULT_DEG[n] / 360.0 * 4095.0 * m.MOTOR_SIGN[n])


def test_shadow_mode_streams_feedback_and_runs_policy(robot):
    ctl, mcu, ports = robot
    time.sleep(1.5)
    s = ctl.status()
    assert s['mode'] == 'shadow' and not s['enabled']
    assert s['servo_ids'] == list(m.MOTOR_IDS) and s['imu_ok']
    assert s['feedback_hz'] > 50 and s['inference_hz'] > 25  # macOS timer jitter allowance
    assert s['gravity'][2] < -0.99
    assert all(abs(a - d) < 0.2 for a, d in zip(s['angles'], m.DEFAULT_DEG))
    assert not any(sv.torque for sv in ports[-1].servos)  # torque stays off in shadow mode


def test_hold_arms_and_sends_the_default_pose(robot):
    ctl, mcu, ports = robot
    with Heartbeat(ctl):
        ctl.set_mode('hold')
        assert wait_for(lambda: ctl.status().get('enabled'), 2)
        bus = ports[-1]
        assert wait_for(lambda: all(sv.torque for sv in bus.servos), 1)
        time.sleep(0.2)
        for n, sv in enumerate(bus.servos):
            assert abs(sv.goal - expected_goal(n)) <= 1, (sv.id, sv.goal, expected_goal(n))


def test_web_heartbeat_loss_disarms(robot):
    ctl, mcu, ports = robot
    with Heartbeat(ctl):
        ctl.set_mode('hold')
        assert wait_for(lambda: ctl.status().get('enabled'), 2)
    # No commands for > 1 s: upstream drops to shadow, the MCU loop turns torque off.
    assert wait_for(lambda: ctl.status()['mode'] == 'shadow', 2.5)
    assert 'heartbeat' in ctl.status()['last_error']
    assert wait_for(lambda: not any(sv.torque for sv in ports[-1].servos), 1)


def test_walk_policy_drives_the_servos(robot):
    ctl, mcu, ports = robot
    with Heartbeat(ctl, twist=(0.2, 0.0, 0.0)):
        ctl.set_mode('policy')
        assert wait_for(lambda: ctl.status().get('enabled'), 2)
        before = ctl.status()['inferred']
        time.sleep(1.0)
        s = ctl.status()
        assert s['mode'] == 'policy' and s['enabled'] and s['recovery']['phase'] == 'walk'
        assert s['inferred'] - before > 25
        assert all(math.isfinite(v) for v in s['policy_targets'])
        moved = [abs(t - d) > 0.5 for t, d in zip(s['policy_targets'], m.DEFAULT_DEG)]
        assert any(moved)  # the walk policy is producing steps, not the static pose
        goals = [sv.goal for sv in ports[-1].servos]
        assert len(set(goals)) > 1


def test_calibration_sets_zero_from_the_encoder(robot, tmp_path):
    ctl, mcu, ports = robot
    bus = ports[-1]
    ctl.set_mode('calibrate')
    assert wait_for(lambda: all(sv.torque and sv.goal == 2047 for sv in bus.servos), 1)
    assert (bus.servos[0].temp_kp, bus.servos[0].temp_kd) == (3, 0)
    ctl.cal_targets_set([2100] * 15)
    assert wait_for(lambda: abs(ctl.status()['raw_pos'][0] - 2100) <= 2, 2)
    result = ctl.cal_finish(0)
    assert abs(result['zero_pos'] - 2100) <= 2
    assert wait_for(lambda: abs(mcu.motors[0].zero_pos - 2100) <= 2, 1)
    saved = json.loads((tmp_path / 'zero_pos.json').read_text())
    assert saved['zero_pos'][0] == result['zero_pos']
    ctl.set_mode('shadow')
    assert wait_for(lambda: not any(sv.torque for sv in bus.servos), 1)
    assert bus.servos[0].temp_kp == 6


def test_usb_unplug_and_replug(robot):
    ctl, mcu, ports = robot
    ports[-1].fail = True
    assert wait_for(lambda: mcu.bus is None, 1)
    assert wait_for(lambda: ctl.status().get('servo_ids') == [], 1)
    assert ctl.status()['imu_ok']  # state keeps flowing, so upstream sees "servos missing"
    assert wait_for(lambda: ctl.status().get('servo_ids') == list(m.MOTOR_IDS), 3)
    assert len(ports) >= 2


def test_servo_id_setup_on_a_fresh_servo(tmp_path, monkeypatch):
    """The Servo setup page flow with one new servo (factory ID 1) on the bus."""
    monkeypatch.setattr(controller_module, 'ZERO_PATH', tmp_path / 'zero_pos.json')
    port = SimSerial(servos=[SimServo(1, 1500)], echo=True)
    mcu = m.VirtualMCU(lambda: port, SimImu())
    ctl = controller_module.Controller(mcu)
    for name, fn in (('duck_state', ctl.receive), ('duck_raw', ctl.receive_raw),
                     ('duck_servo_reply', ctl.receive_servo_reply)):
        mcu.provide(name, fn)
    mcu.start()
    worker = threading.Thread(target=ctl.run, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: ctl.status().get('imu_ok'), 3)
        assert mcu.stats.echo  # adapter echo detected
        ctl.set_mode('servo_debug')
        assert wait_for(lambda: ctl.status().get('mcu_servo_debug'), 1)
        assert ctl.servo_op('ping', 1)['ok']
        assert not ctl.servo_op('ping', 10)['ok']
        assert ctl.servo_op('set_id', 1, new_id=10)['new_id'] == 10
        assert port.servos[0].id == 10 and port.servos[0].lock == 0
        ctl.servo_op('goto', 10, raw_pos=2047)
        assert port.servos[0].torque and port.servos[0].goal == 2047
        ctl.servo_op('set_gains', 10, kp=5, kd=20)
        assert (port.servos[0].perm_kp, port.servos[0].perm_kd) == (5, 20)
        assert wait_for(lambda: abs(ctl.servo_op('read', 10)['raw_pos'] - 2047) <= 2, 2)
        ctl.set_mode('shadow')
        assert wait_for(lambda: not port.servos[0].torque, 1)
    finally:
        ctl.stop()
        worker.join(2)
        mcu.stop()


def test_web_api(robot):
    from xgoduck_pi import web
    from pathlib import Path
    ctl, mcu, ports = robot
    assets = Path(controller_module.__file__).resolve().parents[1] / 'assets'
    server = web.Server(('127.0.0.1', 0), web.make_routes(ctl), assets, mcu=mcu, label='TEST PI')
    web.serve(server)
    base = f'http://127.0.0.1:{server.server_address[1]}'
    try:
        html = urllib.request.urlopen(base + '/').read().decode()
        assert 'TEST PI' in html and 'ARDUINO UNO Q' not in html
        assert b'function' in urllib.request.urlopen(base + '/app.js').read()
        status = json.load(urllib.request.urlopen(base + '/api/status'))
        assert status['pi']['bus_connected'] and status['servo_ids'] == list(m.MOTOR_IDS)

        def post(path, body):
            req = urllib.request.Request(base + path, json.dumps(body).encode(),
                                         {'Content-Type': 'application/json'})
            try:
                with urllib.request.urlopen(req) as r:
                    return r.status, json.load(r)
            except urllib.error.HTTPError as e:
                return e.code, json.load(e)
        assert post('/api/command', {'twist': [0.1, 0, 0]}) == (200, {'ok': True})
        code, body = post('/api/command', {'twist': [9, 0, 0]})
        assert code == 400 and 'bounds' in body['detail']
        assert post('/api/mode', {'mode': 'nope'})[0] == 400
        assert post('/api/mode', {'mode': 'off'}) == (200, {'mode': 'off'})
        req = urllib.request.Request(base + '/../controller.py')
        try:
            urllib.request.urlopen(req)
            assert False, 'path traversal served a file'
        except urllib.error.HTTPError as e:
            assert e.code == 404
    finally:
        server.shutdown()
