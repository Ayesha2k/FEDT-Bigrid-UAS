#!/usr/bin/env python3
"""Single-domain Webots, native gateway, control services and ROSBridge."""
import fcntl
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time

from config import HERE, RUNTIME, load, ports
from services import start_services, await_services
sys.path.insert(0, '/workspace/controller/cf.PyControl/bin')
from launch import Processes, check_ports, select_display


def main():
    config = load('/ds/crazywebotsworld/worlds/crazyflie.wbt')
    headless = os.environ.get('FEDT_HEADLESS','0')=='1'
    if not headless:
        os.environ['DISPLAY'] = select_display(os.environ.get('DISPLAY',''), Path('/host-x11'))
    RUNTIME.mkdir(parents=True,exist_ok=True)
    lock = (RUNTIME / 'supervisor.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    ready = RUNTIME / 'ready.json'
    ready.unlink(missing_ok=True)
    check_ports(ports(config)+[config['webots_port'],config['rosbridge_port']])
    processes = Processes(RUNTIME)
    config['headless'] = headless
    os.environ.update(USER='fedt',ROS_DOMAIN_ID=str(config['ros_domain']))
    try:
        environment = os.environ.copy()
        x11 = Path('/tmp/.X11-unix')
        if headless:
            environment['DISPLAY'] = ':99'
            processes.spawn('xvfb',['Xvfb',':99','-screen','0','1280x720x24','-nolisten','tcp'])
            deadline = time.monotonic()+8
            while not Path('/tmp/.X11-unix/X99').exists():
                processes.check()
                if time.monotonic()>deadline:raise TimeoutError('Private Xvfb unavailable')
                time.sleep(.1)
        else:
            if not x11.exists():x11.symlink_to('/host-x11',target_is_directory=True)
            if shutil.which('glxinfo'):
                graphics = subprocess.run(['glxinfo','-B'],env=environment,capture_output=True,text=True,timeout=10)
                (RUNTIME/'graphics.log').write_text(graphics.stdout+graphics.stderr)
                for line in graphics.stdout.splitlines():
                    if 'Accelerated:' in line or 'OpenGL renderer string:' in line:print(line.strip(),flush=True)
        processes.spawn('webots',['webots',f"--port={config['webots_port']}",'--batch','--mode=realtime',
            '--stdout','--stderr',*(['--no-rendering'] if headless else []),
            '/ds/crazywebotsworld/worlds/crazyflie.wbt'],env=environment)
        sockets = Path(f"/tmp/webots/fedt/{config['webots_port']}/ipc")
        deadline = time.monotonic()+90
        while not sockets.exists():
            processes.check()
            if time.monotonic()>deadline:raise TimeoutError('Webots sockets unavailable; inspect webots.log')
            time.sleep(.2)
        processes.spawn('gateway',['ros2','run','crazyflie_webots_gateway','gateway','--ros-args',
            '-p',f"webots_port:={config['webots_port']}"])
        import rclpy
        from rclpy.node import Node
        from crazyflie_interfaces.srv import AddCrazyflie
        rclpy.init()
        node = Node('fedt_attach_drones')
        try:
            client = node.create_client(AddCrazyflie,'/crazyflie_webots_gateway/add_crazyflie')
            if not client.wait_for_service(timeout_sec=20):raise RuntimeError('Gateway unavailable')
            for drone in config['drone_ids']:
                processes.check()
                request = AddCrazyflie.Request();request.uri=f'webots://{drone}'
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node,future,timeout_sec=15)
                if not future.done() or not future.result().success:raise RuntimeError(f'Could not attach cf{drone}')
        finally:
            node.destroy_node();rclpy.shutdown()
        start_services(processes,config)
        processes.spawn('positions',[sys.executable,str(HERE/'tools/positions.py')])
        processes.spawn('rosbridge',['ros2','launch','rosbridge_server','rosbridge_websocket_launch.xml',
            f"port:={config['rosbridge_port']}",f"address:={config['bind_host']}"])
        await_services(processes,config)
        deadline = time.monotonic()+40
        while True:
            processes.check()
            try:
                with socket.create_connection(('127.0.0.1',config['rosbridge_port']),timeout=.5):pass
                if (RUNTIME/'positions-ready.json').exists():break
            except OSError:pass
            if time.monotonic()>deadline:raise TimeoutError('ROSBridge or aggregated positions unavailable')
            time.sleep(.2)
        temporary=ready.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(config,supervisor_pid=os.getpid()),indent=2)+'\n');temporary.replace(ready)
        print('READY — native ROS domain '+str(config['ros_domain'])+'; drones='+str(config['drone_ids']),flush=True)
        processes.wait()
    except InterruptedError:
        pass
    finally:
        ready.unlink(missing_ok=True)
        processes.close()
        lock.close()


if __name__ == '__main__':
    main()
