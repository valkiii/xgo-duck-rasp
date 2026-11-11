#!/usr/bin/env python3
"""Guided XGO Duck setup. Run on the Mac, then follow the steps in the browser.

    ../.venv/bin/python setup_wizard.py          # opens http://localhost:8790
"""
import argparse
import logging
import signal
import sys
import threading
import webbrowser
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wizard.server import Server, Wizard  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=8790, help='first port to try; the next free one is used if taken')
    ap.add_argument('--no-browser', action='store_true')
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')
    wizard = Wizard()
    for port in range(args.port, args.port + 20):
        try:
            server = Server(wizard, port)
            break
        except OSError:
            continue  # port in use (e.g. another local app)
    else:
        print(f'No free port between {args.port} and {args.port + 19}; pass --port to choose another range.')
        return 1
    url = f'http://localhost:{port}/'
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'XGO Duck setup running at {url}  (Ctrl-C to quit)', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    stop.wait()
    server.shutdown()
    wizard.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
