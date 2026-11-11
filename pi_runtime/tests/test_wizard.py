"""Setup wizard: bench flow on the simulator, remote steps against a local stand-in for the Pi."""
import json
import shutil
import time
import urllib.request
from pathlib import Path

import pytest

from wizard import remote as rmt
from wizard.server import Server, Wizard

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def wiz(tmp_path):
    """A wizard working on a copy of pi_runtime, with the 'Pi' being a local folder."""
    work = tmp_path / 'pi_runtime'
    shutil.copytree(ROOT, work, ignore=shutil.ignore_patterns('upstream', '.pytest_cache', '__pycache__',
                                                              'wizard_state.json'))
    pi_home = tmp_path / 'pi_home'
    pi_home.mkdir()
    fake = {
        'probe': 'echo aarch64; echo "Debian GNU/Linux 12 (bookworm)"; echo "Python 3.11.2"; echo XGO_INSTALLED; echo enabled',
        'i2c': "printf '     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\\n50: -- -- -- -- -- -- -- 57 -- -- -- -- -- -- -- --\\n60: -- -- -- -- -- -- -- -- 68 69 -- -- -- -- -- --\\n'",
        'imu_sample': 'echo \'{"type": "MPU6050", "address": "0x69", "acc": [0.01, 0.99, 0.02], "body_grav": [0, 0, -1], "axes": "-z,-x,+y"}\'',
        'install': 'echo installing; sleep 0.2; echo done',
    }
    w = Wizard(root=work, remote_factory=lambda host: rmt.LocalRemote(pi_home, fake))
    server = Server(w, 0)
    import threading
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f'http://localhost:{server.server_address[1]}'
    yield w, base, pi_home
    server.shutdown()
    w.shutdown()


def call(base, path, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base + path, data, {'Content-Type': 'application/json', 'X-Wizard': '1'})
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.load(r)
    except urllib.error.HTTPError as e:
        return e.code, json.load(e)


def wait_job(base, job):
    for _ in range(200):
        code, j = call(base, f"/api/jobs/{job['id']}")
        if j['status'] != 'running':
            return j
        time.sleep(0.05)
    raise AssertionError('job did not finish')


def test_guards(wiz):
    w, base, _ = wiz
    req = urllib.request.Request(base + '/api/bench/scan', b'{}', {'Content-Type': 'application/json'})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 403  # no X-Wizard header
    req = urllib.request.Request(base + '/api/overview', headers={'Host': 'attacker.example'})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 403  # DNS-rebinding style Host
    assert call(base, '/api/state', {'host': 'a@b; rm -rf /'})[0] == 400


def test_bench_id_flow_persists(wiz):
    w, base, _ = wiz
    assert call(base, '/api/bench/connect', {'device': 'sim:bench'})[1]['echo']
    assert call(base, '/api/bench/scan', {})[1]['ids'] == [1]
    code, r = call(base, '/api/bench/setup', {'old_id': 1, 'new_id': 30})
    assert code == 200 and r['position'] == 2047
    call(base, '/api/state', {'servos_done': [30]})
    assert json.loads((w.root / 'wizard_state.json').read_text())['servos_done'] == [30]
    call(base, '/api/bench/sim_new_servo', {})
    assert call(base, '/api/bench/scan', {})[1]['ids'] == [1]


def test_remote_steps_against_local_pi(wiz):
    w, base, pi_home = wiz
    assert call(base, '/api/pi/probe', {})[0] == 400  # no host yet
    call(base, '/api/state', {'host': 'duck@xgoduck.local'})
    code, p = call(base, '/api/pi/probe', {})
    assert p['ok'] and p['arch_ok'] and p['installed']

    code, job = call(base, '/api/pi/deploy', {})
    assert wait_job(base, job)['status'] == 'ok'
    assert (pi_home / 'xgo-duck' / 'run.py').is_file()
    assert not (pi_home / 'xgo-duck' / 'upstream').exists()  # never shipped: calibration lives there

    j = wait_job(base, call(base, '/api/pi/install', {})[1])
    assert j['status'] == 'ok' and j['lines'] == ['installing', 'done']

    code, i2c = call(base, '/api/pi/i2c', {})
    assert i2c['ok'] and [d['address'] for d in i2c['devices']] == ['0x57', '0x68', '0x69']


def test_imu_capture_and_axes(wiz):
    w, base, _ = wiz
    call(base, '/api/state', {'host': 'duck@xgoduck.local'})
    assert call(base, '/api/pi/imu_axes', {})[0] == 400  # nothing captured yet
    call(base, '/api/pi/imu_capture', {'pose': 'up'})
    # Pretend the second pose came from a 35 deg nose-down tilt with the default mounting.
    poses = dict(w.state.data['imu_poses'], nose_down=[0.0, 0.82, 0.57])
    call(base, '/api/state', {'imu_poses': poses})
    code, ax = call(base, '/api/pi/imu_axes', {})
    assert ax == {'axes': '-z,-x,+y', 'configured': '-z,-x,+y'}
    code, r = call(base, '/api/pi/save_axes', {'axes': '+x,+y,+z'})
    assert r['axes'] == '+x,+y,+z' and 'axes = "+x,+y,+z"' in (w.root / 'config.toml').read_text()
    assert call(base, '/api/pi/save_axes', {'axes': '+x,+y,-z'})[0] == 400  # mirror image refused


def test_i2c_hints():
    assert rmt.describe_i2c([0x57, 0x68])['hint'].startswith('Only the PiSugar')
    assert rmt.describe_i2c([])['hint'].startswith('No IMU')
    assert rmt.describe_i2c([0x69])['ok']


def test_ssh_failure_messages():
    assert 'not found' in rmt.explain_ssh_failure(255, 'ssh: Could not resolve hostname xgoduck.local')
    assert 'Imager' in rmt.explain_ssh_failure(255, 'duck@x: Permission denied (publickey).')
    assert 'booting' in rmt.explain_ssh_failure(255, 'ssh: connect to host x port 22: Operation timed out')
    with pytest.raises(rmt.RemoteError):
        rmt.check_host('-oProxyCommand=evil')
