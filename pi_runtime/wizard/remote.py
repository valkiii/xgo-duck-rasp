"""Running the setup steps on the Pi over SSH, plus a small job runner for long commands.

All remote commands are fixed strings in this file; the only user input is the
host, which is validated before it reaches ssh/rsync.
"""
from __future__ import annotations

import itertools
import json
import os
import re
import subprocess
import threading
import time
from pathlib import Path

HOST_RE = re.compile(r'^(?:[A-Za-z0-9._-]+@)?[A-Za-z0-9][A-Za-z0-9.-]*$')
SSH_OPTS = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'StrictHostKeyChecking=accept-new',
            '-o', 'ServerAliveInterval=15']
RSYNC_EXCLUDES = ['.venv', 'upstream', '__pycache__', '.pytest_cache', 'wizard_state.json']

REMOTE = {
    'probe': 'uname -m; . /etc/os-release 2>/dev/null && echo "$PRETTY_NAME"; python3 --version 2>&1; '
             'test -x xgo-duck/.venv/bin/python && echo XGO_INSTALLED || echo XGO_MISSING; '
             'systemctl is-enabled xgo-duck 2>/dev/null || echo not-installed',
    'install': 'cd xgo-duck && ./install_pi.sh --service',
    'reboot': 'sudo systemctl reboot',
    'i2c': '/usr/sbin/i2cdetect -y 1',  # /usr/sbin is not on a normal user's PATH
    'imu_sample': 'cd xgo-duck && .venv/bin/python tools/imu_check.py --sample',
    'usb': 'ls /dev/serial/by-id/ 2>/dev/null; ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true',
    'bus_scan': 'cd xgo-duck && .venv/bin/python tools/bus_check.py scan',
    'service_start': 'sudo systemctl start xgo-duck && sleep 1 && systemctl is-active xgo-duck',
    'service_stop': 'sudo systemctl stop xgo-duck; systemctl is-active xgo-duck || true',
    'service_restart': 'sudo systemctl restart xgo-duck && sleep 1 && systemctl is-active xgo-duck',
    'service_status': 'systemctl is-active xgo-duck; systemctl is-enabled xgo-duck 2>/dev/null || true',
    'logs': 'journalctl -u xgo-duck -n 150 --no-pager --output=short-iso',
}


class RemoteError(Exception):
    pass


def check_host(host: str) -> str:
    host = (host or '').strip()
    if not HOST_RE.match(host) or host.startswith('-'):
        raise RemoteError('enter the Pi as user@hostname, e.g. duck@xgoduck.local')
    return host


def hostname(host: str) -> str:
    return host.split('@', 1)[-1]


class SshRemote:
    def __init__(self, host: str) -> None:
        self.host = check_host(host)

    def argv(self, key: str) -> list[str]:
        return ['ssh', *SSH_OPTS, self.host, REMOTE[key]]

    def copy_argv(self, local_dir: Path) -> list[str]:
        excludes = [f'--exclude={e}' for e in RSYNC_EXCLUDES]
        return ['rsync', '-az', '--itemize-changes', *excludes, '-e', 'ssh ' + ' '.join(SSH_OPTS),
                f'{local_dir}/', f'{self.host}:xgo-duck/']


class LocalRemote:
    """Runs the 'remote' commands in a local folder; used by the tests."""

    def __init__(self, home: Path, commands: dict | None = None) -> None:
        self.home = Path(home)
        self.host = 'local'
        self.commands = commands or {}

    def argv(self, key: str) -> list[str]:
        return ['bash', '-c', f'cd {self.home} && {self.commands.get(key, REMOTE[key])}']

    def copy_argv(self, local_dir: Path) -> list[str]:
        excludes = [f'--exclude={e}' for e in RSYNC_EXCLUDES]
        (self.home / 'xgo-duck').mkdir(parents=True, exist_ok=True)
        return ['rsync', '-a', '--itemize-changes', *excludes, f'{local_dir}/', f'{self.home}/xgo-duck/']


def run(argv: list[str], timeout: float = 30) -> tuple[int, str]:
    try:
        p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return 124, f'timed out after {timeout:.0f} s'
    except FileNotFoundError as exc:
        return 127, str(exc)
    return p.returncode, (p.stdout + p.stderr).strip()


def explain_ssh_failure(code: int, out: str) -> str:
    text = out.lower()
    if 'could not resolve' in text or 'name or service not known' in text:
        return 'The hostname was not found. Is the Pi powered on and on the same Wi-Fi? Try its IP address.'
    if 'permission denied' in text:
        return ('SSH refused the login. Use the user you set in Raspberry Pi Imager, and add this '
                "Mac's public key there (step 5), or run once in a terminal: ssh-copy-id user@host")
    if 'timed out' in text or 'no route' in text or 'connection refused' in text or code == 124:
        return 'No answer from the Pi. It may still be booting (first boot takes a few minutes), or SSH is off.'
    if 'host key verification failed' in text or 'remote host identification has changed' in text:
        return ('The Pi has a new SSH host key (normal after re-flashing). Remove the old one with: '
                'ssh-keygen -R <hostname>, then try again.')
    return out or f'ssh exited with code {code}'


def probe(remote) -> dict:
    code, out = run(remote.argv('probe'), timeout=20)
    if code != 0:
        return {'ok': False, 'message': explain_ssh_failure(code, out)}
    lines = out.splitlines()
    arch = lines[0] if lines else ''
    info = {'ok': True, 'arch': arch, 'os': lines[1] if len(lines) > 1 else '',
            'python': lines[2] if len(lines) > 2 else '',
            'installed': 'XGO_INSTALLED' in out, 'service': lines[-1] if lines else ''}
    info['arch_ok'] = arch == 'aarch64'
    if not info['arch_ok']:
        info['message'] = f'This Pi runs a {arch} system; flash the 64-bit Raspberry Pi OS (aarch64).'
    return info


def parse_i2cdetect(out: str) -> list[int]:
    found = []
    for line in out.splitlines():
        m = re.match(r'^([0-7]0):\s(.*)$', line.strip())
        if not m:
            continue
        base = int(m.group(1), 16)
        cells = m.group(2).split()
        for col, cell in enumerate(cells):
            if re.fullmatch(r'[0-9a-f]{2}|UU', cell):
                found.append(base + col)
    return found


def describe_i2c(addresses: list[int], imu_address: int = 0x69) -> dict:
    names = {0x69: 'MPU6050 (AD0 high)', 0x68: 'MPU6050 with AD0 low, or a PiSugar 3 clock',
             0x57: 'PiSugar 3', 0x32: 'PiSugar', 0x75: 'PiSugar', 0x6A: 'QMI8658', 0x6B: 'QMI8658'}
    devices = [{'address': f'0x{a:02x}', 'name': names.get(a, 'unknown device')} for a in addresses]
    ok = imu_address in addresses
    hint = ''
    if not ok:
        if 0x68 in addresses and 0x57 not in addresses:
            hint = ('An MPU6050 answers at 0x68 (AD0 not wired). That works too: set address = "0x68" in '
                    'config.toml, or wire AD0 to 3.3 V (pin 17) to move it to 0x69.')
        elif 0x68 in addresses and 0x57 in addresses:
            hint = ('Only the PiSugar shows up. Wire the MPU6050 AD0 pin to 3.3 V (pin 17) so it moves to 0x69, '
                    'and check VCC→pin 1, GND→pin 6, SDA→pin 3, SCL→pin 5.')
        elif 0x68 in addresses:
            hint = 'Something answers at 0x68. If that is the MPU6050, wire AD0 to 3.3 V (pin 17) to move it to 0x69.'
        else:
            hint = 'No IMU found. Check VCC→pin 1 (3.3 V), GND→pin 6, SDA→pin 3, SCL→pin 5 and AD0→pin 17.'
    return {'ok': ok, 'devices': devices, 'hint': hint}


def imu_sample(remote) -> dict:
    code, out = run(remote.argv('imu_sample'), timeout=30)
    try:
        data = json.loads(out.splitlines()[-1])
    except (ValueError, IndexError):
        data = None
    if code != 0 or not isinstance(data, dict) or 'acc' not in data:
        message = data.get('error') if isinstance(data, dict) else explain_ssh_failure(code, out)
        raise RemoteError(message or out)
    return data


# ---- jobs ------------------------------------------------------------------

class Job:
    _ids = itertools.count(1)

    def __init__(self, name: str, argv: list[str], cwd: Path | None = None) -> None:
        self.id = next(self._ids)
        self.name = name
        self.argv = argv
        self.lines: list[str] = []
        self.status = 'running'
        self.code = None
        self.started = time.time()
        self.ended = None
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        try:
            self.proc = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         text=True, bufsize=1, env=env)
        except FileNotFoundError as exc:
            self.proc = None
            self.lines.append(str(exc))
            self.status, self.code, self.ended = 'failed', 127, time.time()
            return
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self) -> None:
        for line in self.proc.stdout:
            self.lines.append(line.rstrip('\n'))
            if len(self.lines) > 5000:
                del self.lines[:1000]
        self.code = self.proc.wait()
        self.ended = time.time()
        self.status = 'ok' if self.code == 0 else 'failed'

    def view(self, since: int = 0) -> dict:
        lines = self.lines[since:]
        return {'id': self.id, 'name': self.name, 'status': self.status, 'code': self.code,
                'lines': lines, 'next': since + len(lines),
                'seconds': round((self.ended or time.time()) - self.started)}

    def stop(self) -> None:
        if self.proc and self.status == 'running':
            self.proc.terminate()


class Jobs:
    def __init__(self) -> None:
        self.jobs: dict[int, Job] = {}
        self.lock = threading.Lock()

    def start(self, name: str, argv: list[str], cwd: Path | None = None) -> Job:
        with self.lock:
            for job in self.jobs.values():
                if job.name == name and job.status == 'running':
                    return job
            job = Job(name, argv, cwd)
            self.jobs[job.id] = job
            return job

    def get(self, job_id: int) -> Job | None:
        return self.jobs.get(job_id)

    def latest(self) -> dict:
        out = {}
        for job in self.jobs.values():
            if job.name not in out or job.id > out[job.name]['id']:
                out[job.name] = {'id': job.id, 'status': job.status}
        return out
