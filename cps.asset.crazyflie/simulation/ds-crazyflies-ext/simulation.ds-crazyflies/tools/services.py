"""Start per-drone HTTP, WebSocket and ordinary diagram services."""
import json
import os
from pathlib import Path
import shutil
import sys
import time
import urllib.request

CF = Path('/workspace/controller/cf.PyControl') if Path('/workspace/controller/cf.PyControl').exists() else Path(__file__).resolve().parents[4] / 'controller/cf.PyControl'


def start_services(processes, config):
    web = processes.directory / 'webview'
    shutil.copytree(CF / 'webview', web, dirs_exist_ok=True, ignore=shutil.ignore_patterns('img', '__pycache__', '*.log'))
    (web / 'runtime-config.js').write_text('window.CF_RUNTIME = ' + json.dumps(config) + ';\n')
    for index, drone in enumerate(config['drone_ids']):
        name = f'cf{drone}'
        processes.spawn(name, [sys.executable, str(CF / 'src/server.py'), '--drone_id', name,
            '--host', config['bind_host'], '--port', str(config['http_port']+index), '--web-root', str(web),
            '--use-sim-time', '--require-command-report', '--wsendpoint', '--wshost', config['bind_host'],
            '--wsport', str(config['ws_port']+index)], env=os.environ.copy())
        processes.spawn('web-'+name, [sys.executable, str(Path(__file__).with_name('web.py')),
            '--directory', str(web), '--host', config['bind_host'], '--port', str(config['web_port']+index)])


def await_services(processes, config):
    pending = set(range(len(config['drone_ids'])))
    deadline = time.monotonic()+100
    while pending:
        processes.check()
        for index in list(pending):
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{config['http_port']+index}/status", timeout=.5) as response:
                    status = json.load(response)
                with urllib.request.urlopen(f"http://127.0.0.1:{config['web_port']+index}/webview.html", timeout=.5) as response:
                    if status.get('connected') and response.status == 200:
                        pending.remove(index)
            except (OSError, ValueError):
                pass
        if time.monotonic()>deadline:
            raise TimeoutError(f'Controllers not ready: {sorted(pending)}')
        processes.stop.wait(.2)
