#!/usr/bin/env python3
"""Shared launch logic. Shell entry points only select their original backend."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time

CF = Path(__file__).resolve().parents[1]
REPO = CF.parents[1]
STACK = REPO / "simulation/ds-crazyflies-ext/simulation.ds-crazyflies"
CONTAINER = "ds-crazyflies-dev"
OWNER = "cps.asset.crazyflie.webots3"


class Processes:
    """Own process groups, log files and signal handling for all launch paths."""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.children = []
        self.files = []
        self.stop = threading.Event()
        signal.signal(signal.SIGINT, lambda *_: self.stop.set())
        signal.signal(signal.SIGTERM, lambda *_: self.stop.set())

    def spawn(self, name, command, env=None, cwd=None):
        output = (self.directory / f"{name}.log").open("ab")
        self.files.append(output)
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                   env=env, cwd=cwd, start_new_session=True)
        self.children.append((name, process))
        return process

    def check(self):
        failed = [(name, p.returncode) for name, p in self.children if p.poll() is not None]
        if failed:
            raise RuntimeError(f"Service exited: {failed}; logs: {self.directory}")
        if self.stop.is_set():
            raise InterruptedError("Startup interrupted")

    def wait(self):
        while not self.stop.wait(.5):
            self.check()

    def close(self):
        for _, p in reversed(self.children):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 10
        for _, p in reversed(self.children):
            try:
                p.wait(timeout=max(.1, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                p.wait()
        for output in self.files:
            output.close()


def check_ports(ports, host="0.0.0.0"):
    probes = []
    try:
        for port in ports:
            probe = socket.socket()
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probes.append(probe)
            try:
                probe.bind((host, port))
            except OSError as exc:
                raise RuntimeError(f"Port {port} is occupied; stop its owning service explicitly") from exc
    finally:
        for probe in probes:
            probe.close()


def select_display(value, x11_root=Path("/tmp/.X11-unix")):
    """Honor a working DISPLAY and resolve stale WSL TCP addresses locally."""
    if value:
        try:
            if value.startswith(":"):
                number = int(value[1:].split(".")[0])
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                    probe.settimeout(1)
                    probe.connect(str(x11_root / f"X{number}"))
            else:
                host, number = value.rsplit(":", 1)
                with socket.create_connection((host, 6000+int(number.split(".")[0])), timeout=1):
                    pass
            return value
        except (OSError, ValueError):
            pass
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
            probe.settimeout(1)
            probe.connect(str(x11_root / "X0"))
        print(f"DISPLAY={value or '(empty)'} unavailable; using the existing local X11 display :0", flush=True)
        return ":0"
    except OSError:
        raise RuntimeError(f"DISPLAY={value or '(empty)'} unavailable; start your display server or add --headless") from None


def docker_command():
    probe = subprocess.run(["docker", "info", "--format", "{{.ServerVersion}}"],
                           capture_output=True, text=True)
    if probe.returncode == 0:
        return ["docker"]
    # Authenticate once and keep all privileged calls in this launcher.
    subprocess.run(["sudo", "-v"], check=True)
    subprocess.run(["sudo", "docker", "info", "--format", "{{.ServerVersion}}"],
                   check=True, stdout=subprocess.DEVNULL)
    return ["sudo", "--preserve-env=DISPLAY,CF_HEADLESS,CF_CONTAINER_RUNTIME,MESA_D3D12_DEFAULT_ADAPTER_NAME", "docker"]


def inspect(docker):
    for _ in range(2):
        result = subprocess.run(docker + ["inspect", CONTAINER], capture_output=True, text=True)
        if result.returncode not in (130, -signal.SIGINT):
            break
    if result.returncode:
        return None
    info = json.loads(result.stdout)[0]
    if info["Config"].get("Labels", {}).get("io.cps.crazyflie.stack") != OWNER:
        raise RuntimeError(f"{CONTAINER} belongs to a different environment; refusing to replace it")
    return info


def docker_start(args):
    docker = docker_command()
    compose = docker + ["compose", "-f", str(STACK / "docker/docker-compose.yaml")]
    if args.action == "build":
        subprocess.run(compose + ["build"], check=True)
        return
    existing = inspect(docker)
    if existing and existing["State"]["Running"]:
        print("Already running; no duplicate services started. http://127.0.0.1:5000/status", flush=True)
        return
    display = "" if args.headless else select_display(os.environ.get("DISPLAY", ""))
    sys.path.insert(0, str(STACK / "tools"))
    from config import load, ports
    config = load()
    check_ports(ports(config) + [config["webots_port"]])
    environment = dict(os.environ, CF_HEADLESS="1" if args.headless else "0", DISPLAY=display)
    runtimes = subprocess.run(docker + ["info", "--format", "{{json .Runtimes}}"], capture_output=True, text=True, check=True)
    if Path("/dev/dxg").exists():
        environment.setdefault("CF_CONTAINER_RUNTIME", "runc")
    elif not args.headless and "nvidia" in json.loads(runtimes.stdout):
        environment.setdefault("CF_CONTAINER_RUNTIME", "nvidia")
    started = False
    owned_id = None
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set())
    signal.signal(signal.SIGTERM, lambda *_: interrupted.set())
    try:
        subprocess.run(compose + ["up", "-d", "--no-build"], env=environment, check=True)
        started = True
        created = inspect(docker)
        owned_id = created["Id"] if created else None
        (STACK / ".runtime/launcher.json").write_text(json.dumps(dict(pid=os.getpid(), owner=OWNER)))
        deadline = time.monotonic()+180
        while not interrupted.wait(.5):
            info = inspect(docker)
            if interrupted.is_set():
                break
            if not info or not info["State"]["Running"]:
                subprocess.run(compose + ["logs", "--tail", "30"], check=False)
                raise RuntimeError("Simulator exited during startup; see the service logs above")
            if info["State"].get("Health", {}).get("Status") == "healthy":
                break
            if time.monotonic() > deadline:
                raise TimeoutError(f"Startup not ready; inspect {STACK / '.runtime'}")
        if not interrupted.is_set():
            ids = ', '.join(f"cf{i}" for i in config["drone_ids"])
            count = len(config["drone_ids"])
            print(f"READY — {ids}\n"
                  f"HTTP: {config['http_port']}–{config['http_port']+count-1}; "
                  f"state diagrams: {config['web_port']}–{config['web_port']+count-1}; "
                  f"wall diagrams: {config['wall_port']}–{config['wall_port']+count-1}\n"
                  "Run your experiment in terminal 2. Ctrl+C stops this simulator.", flush=True)
        while not interrupted.wait(1):
            info = inspect(docker)
            if interrupted.is_set():
                break
            if not info or not info["State"]["Running"]:
                raise RuntimeError("Simulator stopped unexpectedly; inspect .runtime logs")
    finally:
        if started:
            current = inspect(docker)
            if current and current["Id"] == owned_id:
                subprocess.run(compose + ["stop", "--timeout", "20"], env=environment, check=False)
            (STACK / ".runtime/launcher.json").unlink(missing_ok=True)


def local_ros(args, extra):
    # Preserve the original real-ROS entry point without editing its workspace.
    setup = Path(os.environ.get("CF_ROS_SETUP", "/home/crazy/crazyflie_mapping_demo/ros2_ws/install/setup.bash"))
    if not setup.is_file():
        raise RuntimeError(f"ROS workspace missing: {setup}; set CF_ROS_SETUP to your existing setup.bash")
    result = subprocess.run(["bash", "-c", 'source /opt/ros/humble/setup.bash && source "$1" && env -0', "cf-ros", str(setup)],
                            check=True, capture_output=True, env={k: v for k, v in os.environ.items() if k not in
                            {"ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION", "PYTHONPATH", "LD_LIBRARY_PATH",
                             "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH", "ROS_PACKAGE_PATH"}})
    environment = dict(item.split("=", 1) for item in result.stdout.decode().split("\0") if "=" in item)
    environment.update(PYTHONDONTWRITEBYTECODE="1", PYTHONPATH=str(CF / "src") + ":" + environment.get("PYTHONPATH", ""))
    environment["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "30")
    environment["RMW_IMPLEMENTATION"] = os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    import yaml
    configuration = REPO / "simulation/simulation-crazyswarm2/dev_ws/ros2/src/crazyswarm2/crazyflie/config/crazyflies.yaml"
    robots = yaml.safe_load(configuration.read_text())["robots"]
    required = [int(name[2:]) for name, value in robots.items() if value.get("enabled")]
    if not required or any(drone not in range(231, 239) for drone in required):
        raise RuntimeError("Hardware config must enable cf231–cf238; HTTP ports remain fixed at 5000–5007")
    processes = Processes(CF / ".runtime/ros")
    try:
        sys.path.insert(0, str(STACK / "tools"))
        from services import start_services, await_services
        config = dict(drone_ids=list(range(231, 239)), http_port=5000, web_port=8080, ws_port=8765,
                      wall_port=9000, bind_host="0.0.0.0", required_drone_ids=required)
        check_ports([config[k]+i for k in ("http_port", "web_port", "ws_port", "wall_port") for i in range(8)])
        start_services(processes, config, simulated=False, environment=environment, extra=extra)
        await_services(processes, config)
        print(f"READY — hardware ROS services and telemetry: {required}; domain={environment['ROS_DOMAIN_ID']}", flush=True)
        processes.wait()
    finally:
        processes.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("backend", choices=["webots", "ros"])
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--action", choices=["start", "build"], default="start")
    args, extra = parser.parse_known_args()
    try:
        if args.backend == "webots":
            if extra:
                parser.error(f"Unsupported arguments: {' '.join(extra)}")
            docker_start(args)
        else:
            local_ros(args, extra)
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        print(f"Startup failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
