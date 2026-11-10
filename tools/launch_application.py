#!/usr/bin/env python3
"""Launch FEDT or delegate to the independently usable simulator asset."""
import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / 'cps.asset.crazyflie'
SIM_LAUNCHER = ASSET / 'controller/cf.PyControl/bin/launch.py'
STACK = ASSET / 'simulation/ds-crazyflies-ext/simulation.ds-crazyflies'
TARGETS = '-1,-1.5;1,-1.5;1.5,-1;1.5,1;1,1.5;-1,1.5;-1.5,1;-1.5,-1'


def load_launcher():
    spec=importlib.util.spec_from_file_location('fedt_sim_launcher',SIM_LAUNCHER)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def java_environment():
    java = str(Path(os.environ['JAVA_HOME'])/'bin/java') if os.environ.get('JAVA_HOME') else shutil.which('java')
    if not java:raise RuntimeError('JDK 21 is required; set JAVA_HOME to your JDK installation')
    check=subprocess.run([java,'-version'],capture_output=True,text=True,check=True)
    match=re.search(r'version "(\d+)',check.stderr+check.stdout)
    if not match or int(match[1])<21:raise RuntimeError('JDK 21 or newer is required; set JAVA_HOME before launching FEDT')
    return os.environ.copy()


def wait_tcp(processes,port,timeout=120):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        processes.check()
        try:
            with socket.create_connection(('127.0.0.1',port),timeout=.5):return
        except OSError:processes.stop.wait(.3)
    raise TimeoutError(f'Service did not open port {port}')


def wait_http(processes,url,timeout=180):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        processes.check()
        try:
            with urllib.request.urlopen(url,timeout=2) as response:return response.status
        except urllib.error.HTTPError as exc:
            # A root 404 still proves the provider completed HTTP startup.
            if exc.code==404:return exc.code
        except OSError:pass
        processes.stop.wait(.3)
    raise TimeoutError(f'HTTP service unavailable: {url}')


def wait_simulator(processes, config):
    deadline=time.monotonic()+1200
    ready=STACK/'.runtime/ready.json'
    while time.monotonic()<deadline:
        processes.check()
        try:
            metadata=json.loads(ready.read_text())
            if metadata['drone_ids']==config['drone_ids']:
                with urllib.request.urlopen('http://127.0.0.1:5007/status',timeout=2) as response:
                    if json.load(response).get('connected'):return
        except (OSError,ValueError,KeyError):pass
        processes.stop.wait(.3)
    raise TimeoutError('Simulator did not become ready')


class Monitor:
    def __init__(self,directory):
        self.socket='fedt-'+str(os.getpid())
        self.session='FEDT'
        self.count=0
        self.directory=directory

    def command(self,*args,**kwargs):
        return subprocess.run(['tmux','-L',self.socket,*args],**kwargs)

    def add(self,title):
        command=shlex.join(['tail','-n','+1','-F',str(self.directory/(title+'.log'))])
        if self.count==0:
            self.command('new-session','-d','-s',self.session,'-x','200','-y','45',command,check=True)
            self.command('set-option','-t',self.session,'pane-border-status','top',check=True)
            self.command('set-option','-t',self.session,'pane-border-format','#{pane_title}',check=True)
            self.command('bind-key','-n','C-q','kill-session','-t',self.session,check=True)
        else:
            self.command('split-window','-t',self.session,command,check=True)
            self.command('select-layout','-t',self.session,'tiled',stdout=subprocess.DEVNULL,check=True)
        self.command('select-pane','-t',self.session,'-T',title,check=True)
        self.count+=1

    def alive(self):
        return self.command('has-session','-t',self.session,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0

    def close(self):
        self.command('kill-server',stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-only',action='store_true',help='Run only the standalone simulator; no Java or tmux required')
    parser.add_argument('--headless',action='store_true',help='Run Webots without its GUI')
    args=parser.parse_args()
    if args.sim_only:
        os.execv(sys.executable,[sys.executable,str(SIM_LAUNCHER),*(['--headless'] if args.headless else [])])
    if not shutil.which('tmux'):parser.exit(1,'tmux is required for the full system\n')
    try:environment=java_environment()
    except (RuntimeError,OSError,subprocess.CalledProcessError) as exc:parser.exit(1,str(exc)+'\n')
    app=ROOT/'bigraphs.drone-collision-free'
    provider=ROOT/'bigrid-provider-service'
    for directory in (app,provider):
        if not (directory/'mvnw').is_file():parser.exit(1,f'Missing {directory}; use --sim-only for a simulator-only checkout\n')
    launcher=load_launcher()
    sys.path.insert(0,str(STACK/'tools'))
    from config import load,ports
    config=load()
    if config['drone_ids']!=list(range(231,239)):parser.exit(1,'The FEDT mission expects cf231–cf238; standalone simulation supports other world files\n')
    runtime=ROOT/'.runtime'
    runtime.mkdir(exist_ok=True)
    lock=(runtime/'application.lock').open('w')
    processes=None;monitor=None;attached=None
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        launcher.check_ports(ports(config)+[config['webots_port'],config['rosbridge_port'],2036,7070,8090])
        processes=launcher.Processes(runtime)
        signal.signal(signal.SIGHUP,lambda *_:processes.stop.set())
        monitor=Monitor(runtime)
        def spawn(name,command,cwd=None):
            processes.spawn(name,command,env=environment,cwd=cwd);monitor.add(name)
        spawn('simulation',[sys.executable,str(SIM_LAUNCHER),*(['--headless'] if args.headless else [])])
        wait_simulator(processes,config)
        spawn('cdo',['bash','./mvnw','test','-DrunDisabledTests=true','-Dtest=CDOStandaloneServerTest#run_server_test_01'],app)
        wait_tcp(processes,2036,300)
        spawn('grid',['bash','./mvnw','spring-boot:run','-Dspring-boot.run.arguments=--server.port=7070'],provider)
        wait_http(processes,'http://127.0.0.1:7070/',300)
        arguments=' '.join(['--drone.count=8','--drone.targets='+TARGETS,'--bigrid.service.base-url=http://127.0.0.1:7070',
                            '--drone.control.base-url=http://127.0.0.1','--drone.control.start-port=5000',
                            '--ros.bridge.host=127.0.0.1','--ros.sim.mode=true','--fault.injection.port=8090'])
        spawn('application',['bash','./mvnw','spring-boot:run','-Dspring-boot.run.mainClass=org.example.Application3D',
                             '-Dspring-boot.run.arguments='+arguments],app)
        print('FEDT services started. Ctrl+Q closes this private tmux session and its owned services.',flush=True)
        if sys.stdin.isatty():
            attached=subprocess.Popen(['tmux','-L',monitor.socket,'attach-session','-t',monitor.session])
        else:
            print('Attach with: '+shlex.join(['tmux','-L',monitor.socket,'attach-session','-t',monitor.session]),flush=True)
        while not processes.stop.wait(.5):
            processes.check()
            if not monitor.alive() or attached and attached.poll() is not None:break
        return 0
    except BlockingIOError:
        print('A FEDT launcher already owns this workspace',file=sys.stderr);return 1
    except InterruptedError:
        return 0
    except (OSError,RuntimeError,subprocess.CalledProcessError) as exc:
        print(f'FEDT startup failed: {exc}; inspect {runtime}',file=sys.stderr);return 1
    finally:
        if processes:processes.close()
        if monitor:monitor.close()
        if attached:attached.wait(timeout=5)
        lock.close()


if __name__=='__main__':
    raise SystemExit(main())
