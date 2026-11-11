"""The setup wizard's local web server (runs on the Mac, listens on 127.0.0.1 only)."""
from __future__ import annotations

import json
import logging
import mimetypes
import re
import socketserver
import subprocess
import sys
import threading
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from . import bench as bench_mod
from . import remote as rmt
from xgoduck_pi.imu import axes_from_poses, parse_axes

log = logging.getLogger('wizard')
ROOT = Path(__file__).resolve().parents[1]          # pi_runtime/
STATIC = Path(__file__).resolve().parent / 'static'
SIM_PORT = 9600


class State:
    """Progress that survives restarts: wizard_state.json next to config.toml."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.lock = threading.Lock()
        try:
            self.data = json.loads(path.read_text())
        except (OSError, ValueError):
            self.data = {}
        self.data.setdefault('checklist', {})
        self.data.setdefault('servos_done', [])
        self.data.setdefault('steps_done', [])
        self.data.setdefault('host', '')
        self.data.setdefault('imu_poses', {})

    def update(self, patch: dict) -> dict:
        allowed = {'checklist', 'servos_done', 'steps_done', 'host', 'imu_poses', 'notes'}
        with self.lock:
            for key, value in patch.items():
                if key not in allowed:
                    raise ValueError(f'unknown state key {key}')
                if key == 'host' and value:
                    value = rmt.check_host(value)
                self.data[key] = value
            self.path.write_text(json.dumps(self.data, indent=2))
            return dict(self.data)


class Wizard:
    def __init__(self, root: Path = ROOT, remote_factory=None) -> None:
        self.root = root
        self.state = State(root / 'wizard_state.json')
        self.bench = bench_mod.Bench()
        self.jobs = rmt.Jobs()
        self.remote_factory = remote_factory or rmt.SshRemote
        self.sim_proc: subprocess.Popen | None = None

    # ---- helpers ----------------------------------------------------------
    def remote(self):
        host = self.state.data.get('host')
        if not host:
            raise ValueError('enter the Pi address in step 5 first')
        return self.remote_factory(host)

    def config_axes(self) -> str:
        text = (self.root / 'config.toml').read_text()
        m = re.search(r'^axes\s*=\s*"([^"]*)"', text, re.M)
        return m.group(1) if m else '-z,-x,+y'

    def write_axes(self, axes: str) -> None:
        parse_axes(axes)  # validates
        path = self.root / 'config.toml'
        text = path.read_text()
        new, n = re.subn(r'^axes\s*=\s*"[^"]*"', f'axes = "{axes}"', text, count=1, flags=re.M)
        if n != 1:
            raise ValueError('could not find the axes line in config.toml')
        path.write_text(new)

    def ssh_key(self) -> dict:
        for name in ('id_ed25519.pub', 'id_ecdsa.pub', 'id_rsa.pub'):
            p = Path.home() / '.ssh' / name
            if p.is_file():
                return {'path': str(p), 'key': p.read_text().strip()}
        return {'path': '', 'key': ''}

    def make_ssh_key(self) -> dict:
        existing = self.ssh_key()
        if existing['key']:
            return existing
        target = Path.home() / '.ssh' / 'id_ed25519'
        if target.exists():
            raise ValueError(f'{target} exists without a .pub file; not touching it')
        target.parent.mkdir(mode=0o700, exist_ok=True)
        code, out = rmt.run(['ssh-keygen', '-t', 'ed25519', '-N', '', '-f', str(target), '-C', 'xgo-duck-setup'])
        if code != 0:
            raise ValueError(out)
        return self.ssh_key()

    def runtime_status(self) -> dict:
        host = self.state.data.get('host')
        if not host:
            return {'reachable': False}
        url = f'http://{rmt.hostname(host)}:9527/api/status'
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                s = json.load(r)
        except Exception as exc:
            return {'reachable': False, 'url': url, 'error': str(exc)}
        return {'reachable': True, 'url': url, 'mode': s.get('mode'), 'servos': len(s.get('servo_ids') or []),
                'missing': s.get('missing_joint_ids'), 'imu_ok': s.get('imu_ok'),
                'feedback_hz': s.get('feedback_hz'), 'inference_hz': s.get('inference_hz'),
                'last_error': s.get('last_error'), 'gravity': s.get('gravity'),
                'overruns': s.get('mcu_overruns'), 'pi': s.get('pi')}

    # ---- simulator --------------------------------------------------------
    def sim_status(self) -> dict:
        running = self.sim_proc is not None and self.sim_proc.poll() is None
        return {'running': running, 'url': f'http://localhost:{SIM_PORT}/' if running else ''}

    def sim_start(self) -> dict:
        if not self.sim_status()['running']:
            self.sim_proc = subprocess.Popen(
                [sys.executable, str(self.root / 'run.py'), '--sim', '--http-port', str(SIM_PORT)],
                cwd=self.root, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return self.sim_status()

    def sim_stop(self) -> dict:
        if self.sim_proc is not None and self.sim_proc.poll() is None:
            self.sim_proc.terminate()
            try:
                self.sim_proc.wait(5)
            except subprocess.TimeoutExpired:
                self.sim_proc.kill()
        self.sim_proc = None
        return self.sim_status()

    def shutdown(self) -> None:
        self.sim_stop()
        self.bench.disconnect()

    # ---- routes -------------------------------------------------------------
    def routes(self):
        b = self.bench

        def remote_cmd(key, timeout=30):
            code, out = rmt.run(self.remote().argv(key), timeout=timeout)
            return {'ok': code == 0, 'code': code, 'output': out,
                    'message': '' if code == 0 else rmt.explain_ssh_failure(code, out)}

        def job(name, argv, cwd=None):
            return self.jobs.start(name, argv, cwd).view()

        def imu_axes(body):
            poses = self.state.data.get('imu_poses', {})
            if 'up' not in poses or 'nose_down' not in poses:
                raise ValueError('capture both poses first')
            return {'axes': axes_from_poses(poses['up'], poses['nose_down']), 'configured': self.config_axes()}

        def imu_capture(body):
            pose = body.get('pose')
            if pose not in ('up', 'nose_down'):
                raise ValueError('pose must be up or nose_down')
            sample = rmt.imu_sample(self.remote())
            poses = dict(self.state.data.get('imu_poses', {}))
            poses[pose] = sample['acc']
            self.state.update({'imu_poses': poses})
            return {'sample': sample, 'poses': poses}

        def i2c(body):
            r = remote_cmd('i2c')
            if not r['ok']:
                return r
            return dict(r, **rmt.describe_i2c(rmt.parse_i2cdetect(r['output'])))

        return {
            ('GET', '/api/overview'): lambda body: {
                'state': self.state.data, 'bench': b.status(), 'sim': self.sim_status(),
                'jobs': self.jobs.latest(), 'axes': self.config_axes(), 'joints': bench_mod.JOINTS},
            ('POST', '/api/state'): lambda body: self.state.update(body),
            ('GET', '/api/ports'): lambda body: {'ports': bench_mod.list_ports()},
            ('POST', '/api/bench/connect'): lambda body: b.connect(body['device']),
            ('POST', '/api/bench/disconnect'): lambda body: b.disconnect(),
            ('POST', '/api/bench/scan'): lambda body: b.scan(),
            ('POST', '/api/bench/read_all'): lambda body: b.read_all(),
            ('POST', '/api/bench/setup'): lambda body: b.setup(int(body['old_id']), int(body['new_id'])),
            ('POST', '/api/bench/wiggle'): lambda body: b.wiggle(int(body['id'])),
            ('POST', '/api/bench/sim_new_servo'): lambda body: b.sim_new_servo(),
            ('GET', '/api/sshkey'): lambda body: self.ssh_key(),
            ('POST', '/api/sshkey/create'): lambda body: self.make_ssh_key(),
            ('POST', '/api/pi/probe'): lambda body: rmt.probe(self.remote()),
            ('POST', '/api/pi/deploy'): lambda body: job('deploy', self.remote().copy_argv(self.root)),
            ('POST', '/api/pi/install'): lambda body: job('install', self.remote().argv('install')),
            ('POST', '/api/pi/reboot'): lambda body: dict(remote_cmd('reboot', 15), ok=True),
            ('POST', '/api/pi/i2c'): i2c,
            ('POST', '/api/pi/imu_capture'): imu_capture,
            ('POST', '/api/pi/imu_axes'): imu_axes,
            ('POST', '/api/pi/save_axes'): lambda body: (self.write_axes(body['axes']),
                                                          {'ok': True, 'axes': self.config_axes()})[1],
            ('POST', '/api/pi/imu_live'): lambda body: rmt.imu_sample(self.remote()),
            ('POST', '/api/pi/usb'): lambda body: remote_cmd('usb'),
            ('POST', '/api/pi/bus_scan'): lambda body: remote_cmd('bus_scan', 20),
            ('POST', '/api/pi/service'): lambda body: remote_cmd(
                {'start': 'service_start', 'stop': 'service_stop', 'restart': 'service_restart',
                 'status': 'service_status'}[body['action']], 40),
            ('POST', '/api/pi/logs'): lambda body: remote_cmd('logs'),
            ('GET', '/api/pi/runtime'): lambda body: self.runtime_status(),
            ('POST', '/api/sim/start'): lambda body: self.sim_start(),
            ('POST', '/api/sim/stop'): lambda body: self.sim_stop(),
        }


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, wizard: Wizard, port: int) -> None:
        self.wizard = wizard
        self.routes = wizard.routes()
        super().__init__(('127.0.0.1', port), Handler)

    def server_bind(self):
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = self.server_address[:2]


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server: Server

    def log_message(self, fmt, *args):
        log.debug(fmt, *args)

    def _send(self, status, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status, payload) -> None:
        self._send(status, json.dumps(payload, default=str).encode(), 'application/json')

    def _local_host(self) -> bool:
        # Blocks DNS-rebinding: only accept requests addressed to this machine by name or IP.
        host = (self.headers.get('Host') or '').rsplit(':', 1)[0]
        return host in ('localhost', '127.0.0.1')

    def _api(self, method: str) -> None:
        length = int(self.headers.get('Content-Length') or 0)
        raw = self.rfile.read(length) if length else b''
        if not self._local_host():
            self._json(HTTPStatus.FORBIDDEN, {'detail': 'local access only'})
            return
        if method == 'POST' and self.headers.get('X-Wizard') != '1':
            # A custom header cannot be sent cross-site without CORS approval, which we never give.
            self._json(HTTPStatus.FORBIDDEN, {'detail': 'missing X-Wizard header'})
            return
        path = self.path.split('?', 1)[0]
        query = self.path.split('?', 1)[1] if '?' in self.path else ''
        if method == 'GET' and path.startswith('/api/jobs/'):
            self._job(path, query)
            return
        route = self.server.routes.get((method, path))
        if route is None:
            self._json(HTTPStatus.NOT_FOUND, {'detail': 'not found'})
            return
        try:
            body = json.loads(raw) if raw.strip() else {}
            self._json(HTTPStatus.OK, route(body))
        except (KeyError, TypeError, ValueError, bench_mod.BenchError, rmt.RemoteError) as exc:
            self._json(HTTPStatus.BAD_REQUEST, {'detail': str(exc).strip("'")})
        except Exception as exc:
            log.exception('%s %s failed', method, path)
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {'detail': str(exc)})

    def _job(self, path, query) -> None:
        try:
            job_id = int(path.rsplit('/', 1)[1])
            since = int(dict(p.split('=', 1) for p in query.split('&') if '=' in p).get('since', 0))
        except ValueError:
            self._json(HTTPStatus.BAD_REQUEST, {'detail': 'bad job id'})
            return
        job = self.server.wizard.jobs.get(job_id)
        if job is None:
            self._json(HTTPStatus.NOT_FOUND, {'detail': 'no such job'})
        else:
            self._json(HTTPStatus.OK, job.view(since))

    def do_GET(self):
        if self.path.startswith('/api/'):
            self._api('GET')
            return
        if not self._local_host():
            self._send(HTTPStatus.FORBIDDEN, b'local access only', 'text/plain')
            return
        name = self.path.split('?', 1)[0].lstrip('/') or 'index.html'
        target = (STATIC / name).resolve()
        if STATIC.resolve() not in target.parents or not target.is_file():
            self._send(HTTPStatus.NOT_FOUND, b'not found', 'text/plain')
            return
        ctype = mimetypes.guess_type(target.name)[0] or 'application/octet-stream'
        if ctype.startswith('text/') or ctype.endswith('javascript'):
            ctype += '; charset=utf-8'
        self._send(HTTPStatus.OK, target.read_bytes(), ctype)

    def do_POST(self):
        self._api('POST')
