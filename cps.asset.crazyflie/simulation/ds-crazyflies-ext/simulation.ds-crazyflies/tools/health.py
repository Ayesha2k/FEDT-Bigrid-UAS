"""Ready only while the supervisor, all controllers and ROSBridge are alive."""
import json
import os
from pathlib import Path
import socket
import urllib.request


def main():
    root = Path('/ds/ds-crazyflies/.runtime')
    ready = json.loads((root / 'ready.json').read_text())
    os.kill(ready['supervisor_pid'], 0)
    if not (root / 'positions-ready.json').exists():
        raise RuntimeError('Position aggregation unavailable')
    for index, drone in enumerate(ready['drone_ids']):
        with urllib.request.urlopen(f"http://127.0.0.1:{ready['http_port']+index}/status", timeout=.3) as response:
            status = json.load(response)
        if not status.get('connected') or status.get('drone_id') != f'cf{drone}':
            raise RuntimeError('Drone telemetry unavailable')
    with socket.create_connection(('127.0.0.1',ready['rosbridge_port']), timeout=.5):
        pass


if __name__ == '__main__':
    main()
