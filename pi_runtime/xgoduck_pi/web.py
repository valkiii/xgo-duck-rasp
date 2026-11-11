"""The upstream web UI and its /api routes, without the Arduino App Lab WebUI brick.

Routes, request bodies and error replies ({"detail": ...} with HTTP 400) follow
upstream python/main.py, so assets/app.js, calibrate.js and servo.js work as-is.
"""
from __future__ import annotations

import json
import logging
import mimetypes
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

log = logging.getLogger('xgoduck.web')


def make_routes(controller):
    c = controller

    def servo(body):
        return c.servo_op(body['action'], body['target_id'], new_id=body.get('new_id'),
                          kp=body.get('kp', 5), kd=body.get('kd', 20), raw_pos=body.get('raw_pos', 2047))

    def filtering(body):
        leg, head = c.set_action_alphas(**body)
        return {'leg': leg, 'head': head}

    return {
        ('GET', '/api/status'): lambda body: c.status(),
        ('POST', '/api/mode'): lambda body: c.set_mode(body['mode']),
        ('POST', '/api/command'): lambda body: c.command(**body),
        ('POST', '/api/filter'): filtering,
        ('POST', '/api/pick'): lambda body: c.start_pick(),
        ('POST', '/api/roulade'): lambda body: c.start_roulade(),
        ('POST', '/api/cal/enter'): lambda body: c.set_mode('calibrate'),
        ('POST', '/api/cal/exit'): lambda body: c.set_mode('shadow'),
        ('POST', '/api/cal/targets'): lambda body: c.cal_targets_set(body['targets']),
        ('POST', '/api/cal/finish'): lambda body: c.cal_finish(body['index']),
        ('POST', '/api/servo/enter'): lambda body: c.set_mode('servo_debug'),
        ('POST', '/api/servo/exit'): lambda body: c.set_mode('shadow'),
        ('POST', '/api/servo'): servo,
    }


def _json_default(value):
    try:  # numpy scalars and arrays
        return value.tolist()
    except AttributeError:
        return str(value)


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, routes, assets: Path, mcu=None, label: str = 'RASPBERRY PI ZERO 2 W'):
        self.routes = routes
        self.assets = assets.resolve()
        self.mcu = mcu
        self.label = label
        super().__init__(address, Handler)

    def server_bind(self):
        # HTTPServer.server_bind() does a reverse DNS lookup that can stall for seconds.
        import socketserver
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = self.server_address[:2]


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server: Server

    def log_message(self, fmt, *args):
        log.debug('%s %s', self.address_string(), fmt % args)

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def _json(self, status: int, payload) -> None:
        self._send(status, json.dumps(payload, default=_json_default).encode(), 'application/json')

    def _api(self, method: str) -> None:
        path = self.path.split('?', 1)[0]
        route = self.server.routes.get((method, path))
        length = int(self.headers.get('Content-Length') or 0)
        raw = self.rfile.read(length) if length else b''
        if route is None:
            self._json(HTTPStatus.NOT_FOUND, {'detail': 'Not Found'})
            return
        try:
            body = json.loads(raw) if raw.strip() else {}
            if not isinstance(body, dict):
                raise ValueError('JSON object expected')
            result = route(body)
            if path == '/api/status' and self.server.mcu is not None:
                result['pi'] = pi_status(self.server.mcu)
            self._json(HTTPStatus.OK, result)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._json(HTTPStatus.BAD_REQUEST, {'detail': str(exc)})
        except Exception as exc:  # keep the server alive; surface the error in the UI
            log.exception('API %s %s failed', method, path)
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {'detail': str(exc)})

    def _static(self) -> None:
        path = self.path.split('?', 1)[0]
        name = 'index.html' if path in ('', '/') else path.lstrip('/')
        target = (self.server.assets / name).resolve()
        if self.server.assets not in target.parents or not target.is_file():
            self._send(HTTPStatus.NOT_FOUND, b'not found', 'text/plain')
            return
        body = target.read_bytes()
        if target.suffix == '.html':
            body = body.replace(b'ARDUINO UNO Q', self.server.label.encode())
        content_type = mimetypes.guess_type(target.name)[0] or 'application/octet-stream'
        if content_type.startswith('text/') or content_type.endswith('javascript'):
            content_type += '; charset=utf-8'
        self._send(HTTPStatus.OK, body, content_type)

    def do_GET(self):
        if self.path.startswith('/api/'):
            self._api('GET')
        else:
            self._static()

    def do_HEAD(self):
        self._static()

    def do_POST(self):
        self._api('POST')


def pi_status(mcu) -> dict:
    s = mcu.stats
    imu = mcu.imu
    return dict(bus_connected=mcu.bus is not None, bus_error=s.bus_error, bus_errors=s.bus_errors,
                adapter_echo=s.echo, imu_type=type(imu).__name__, imu_error=getattr(imu, 'last_error', ''),
                tick_overruns=s.overruns, report_drops=s.report_drops)


def serve(server: Server) -> threading.Thread:
    t = threading.Thread(target=server.serve_forever, name='http', daemon=True)
    t.start()
    return t
