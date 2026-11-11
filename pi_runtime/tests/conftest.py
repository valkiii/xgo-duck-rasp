import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / 'upstream' / 'python'
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(UPSTREAM))

if not (UPSTREAM / 'controller.py').is_file():
    raise RuntimeError('run ./fetch_upstream.sh before the tests')


def wait_for(predicate, timeout=3.0, step=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


@pytest.fixture
def robot(tmp_path, monkeypatch):
    """Upstream Controller + VirtualMCU on a simulated bus and IMU, as run.py wires them."""
    import controller as controller_module
    from xgoduck_pi.mcu import VirtualMCU, ZERO_POS, DEFAULT_DEG
    from xgoduck_pi.sim import SimSerial, SimImu

    monkeypatch.setattr(controller_module, 'ZERO_PATH', tmp_path / 'zero_pos.json')
    ports = []

    def open_port():
        port = SimSerial(zero_pos=ZERO_POS, default_deg=DEFAULT_DEG)
        ports.append(port)
        return port

    mcu = VirtualMCU(open_port, SimImu(), echo='auto')
    ctl = controller_module.Controller(mcu)
    for name, fn in (('duck_state', ctl.receive), ('duck_boot_pd', ctl.receive_boot_pd),
                     ('duck_raw', ctl.receive_raw), ('duck_servo_reply', ctl.receive_servo_reply)):
        mcu.provide(name, fn)
    mcu.start()
    import threading
    worker = threading.Thread(target=ctl.run, daemon=True)
    worker.start()
    assert wait_for(lambda: ctl.status().get('full_robot_ready'), 5), ctl.status().get('last_error')
    yield ctl, mcu, ports
    ctl.stop()
    worker.join(2)
    mcu.stop()
