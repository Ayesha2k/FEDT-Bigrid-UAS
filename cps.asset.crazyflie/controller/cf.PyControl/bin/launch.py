#!/usr/bin/env python3
"""Launch the standalone FEDT simulator from this asset's source tree."""
import argparse
import fcntl
import hashlib
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
ASSET = CF.parents[1]
STACK = ASSET / "simulation/ds-crazyflies-ext/simulation.ds-crazyflies"
CONTAINER = "fedt-crazyflies-dev"
OWNER = "fedt-bigrid-uas.webots"
SOURCE_ID = hashlib.sha256(str(ASSET).encode()).hexdigest()[:16]


class Processes:
    """Track only child process groups created by this supervisor."""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.children = []
        self.files = []
        self.stop = threading.Event()
        signal.signal(signal.SIGINT, lambda *_: self.stop.set())
        signal.signal(signal.SIGTERM, lambda *_: self.stop.set())
        signal.signal(signal.SIGHUP, lambda *_: self.stop.set())

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
            raise RuntimeError(f"Service exited: {failed}; inspect {self.directory}")
        if self.stop.is_set():
            raise InterruptedError("Startup interrupted")

    def wait(self):
        while not self.stop.wait(.5):
            self.check()

    def close(self):
        for _, process in reversed(self.children):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 35
        for _, process in reversed(self.children):
            try:
                process.wait(timeout=max(.1, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        for output in self.files:
            output.close()


def check_ports(ports):
    probes = []
    try:
        for port in ports:
            probe = socket.socket()
            probes.append(probe)
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("0.0.0.0", port))
            except OSError as exc:
                raise RuntimeError(f"Port {port} is occupied; stop its owning service explicitly") from exc
    finally:
        for probe in probes:
            probe.close()


def select_display(value, x11_root=Path("/tmp/.X11-unix")):
    if value:
        try:
            if value.startswith(":"):
                number = int(value[1:].split(".")[0])
                with socket.socket(socket.AF_UNIX) as probe:
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
        with socket.socket(socket.AF_UNIX) as probe:
            probe.settimeout(1)
            probe.connect(str(x11_root / "X0"))
        print("Using local X11 display :0", flush=True)
        return ":0"
    except OSError:
        raise RuntimeError("No X11 display is available; use --headless") from None


def graphics_override(headless, runtimes, dxg=None, dri=None, authority=None):
    """Generate platform-only Compose settings; no global display/driver changes."""
    service = {"environment": {}, "volumes": [], "devices": []}
    if headless:
        return {"services": {"webots": service}}
    service["volumes"].append("/tmp/.X11-unix:/host-x11:ro")
    dxg = Path("/dev/dxg").exists() if dxg is None else dxg
    dri = Path("/dev/dri").exists() if dri is None else dri
    if dxg:
        service["devices"].append("/dev/dxg:/dev/dxg")
        service["volumes"].append("/usr/lib/wsl:/usr/lib/wsl:ro")
        service["environment"].update(LD_LIBRARY_PATH="/usr/lib/wsl/lib",
            MESA_D3D12_DEFAULT_ADAPTER_NAME=os.environ.get("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA"))
    elif "nvidia" in runtimes:
        service["runtime"] = "nvidia"
        service["environment"].update(NVIDIA_VISIBLE_DEVICES="all", NVIDIA_DRIVER_CAPABILITIES="all")
    elif dri:
        service["devices"].append("/dev/dri:/dev/dri")
    authority = authority or os.environ.get("XAUTHORITY") or str(Path.home() / ".Xauthority")
    if Path(authority).is_file():
        service["volumes"].append(f"{Path(authority).resolve()}:/run/fedt.xauthority:ro")
        service["environment"]["XAUTHORITY"] = "/run/fedt.xauthority"
    return {"services": {"webots": service}}


def docker_command():
    probe = subprocess.run(["docker", "info", "--format", "{{.ServerVersion}}"], capture_output=True)
    if probe.returncode == 0:
        return ["docker"]
    subprocess.run(["sudo", "-v"], check=True)
    subprocess.run(["sudo", "docker", "info"], check=True, stdout=subprocess.DEVNULL)
    return ["sudo", "--preserve-env=DISPLAY,FEDT_HEADLESS,FEDT_IMAGE,FEDT_SOURCE_ID", "docker"]


def inspect(docker):
    result = subprocess.run(docker + ["inspect", CONTAINER], capture_output=True, text=True)
    if result.returncode:
        return None
    info = json.loads(result.stdout)[0]
    labels = info["Config"].get("Labels", {})
    if labels.get("io.fedt.stack") != OWNER or labels.get("io.fedt.source") != SOURCE_ID:
        raise RuntimeError(f"{CONTAINER} belongs to another source tree; refusing to replace it")
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--action", choices=["start", "build"], default="start")
    args = parser.parse_args()
    sys.path.insert(0, str(STACK / "tools"))
    from config import load, ports
    runtime = STACK / ".runtime"
    runtime.mkdir(parents=True, exist_ok=True)
    lock = (runtime / "launcher.lock").open("w")
    owned_id = None
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        docker = docker_command()
        env = dict(os.environ, FEDT_SOURCE_ID=SOURCE_ID, FEDT_HEADLESS=str(int(args.headless)))
        compose = docker + ["compose", "-f", str(STACK / "docker/docker-compose.yaml")]
        if args.action == "build":
            subprocess.run(compose + ["build"], env=env, check=True)
            return 0
        existing = inspect(docker)
        if existing and existing["State"]["Running"]:
            print("Already running: http://127.0.0.1:5000/status", flush=True)
            return 0
        config = load()
        check_ports(ports(config) + [config["webots_port"], config["rosbridge_port"]])
        env["DISPLAY"] = "" if args.headless else select_display(env.get("DISPLAY", ""))
        runtimes = json.loads(subprocess.check_output(docker + ["info", "--format", "{{json .Runtimes}}"], text=True))
        override = runtime / "compose.graphics.json"
        override.write_text(json.dumps(graphics_override(args.headless, runtimes), indent=2))
        compose += ["-f", str(override)]
        image = env.get("FEDT_IMAGE", "fedt-crazyflies:humble-webots-r2025a")
        if subprocess.run(docker + ["image", "inspect", image], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode:
            print("Building the simulator from this source tree...", flush=True)
            subprocess.run(compose + ["build"], env=env, check=True)
        stopped = threading.Event()
        signal.signal(signal.SIGINT, lambda *_: stopped.set())
        signal.signal(signal.SIGTERM, lambda *_: stopped.set())
        signal.signal(signal.SIGHUP, lambda *_: stopped.set())
        subprocess.run(compose + ["up", "-d", "--no-build"], env=env, check=True)
        owned_id = inspect(docker)["Id"]
        deadline = time.monotonic()+180
        while not stopped.wait(.5):
            info = inspect(docker)
            if not info or not info["State"]["Running"]:
                subprocess.run(docker + ["logs", "--tail", "30", owned_id], check=False)
                raise RuntimeError("Simulator exited during startup")
            if info["State"].get("Health", {}).get("Status") == "healthy":
                break
            if time.monotonic() > deadline:
                raise TimeoutError(f"Startup timed out; inspect {runtime}")
        if not stopped.is_set():
            print("READY — " + ", ".join(f"cf{i}" for i in config["drone_ids"])
                  + f"; HTTP {config['http_port']}; ROSBridge {config['rosbridge_port']}", flush=True)
        while not stopped.wait(1):
            info = inspect(docker)
            if not info or not info["State"]["Running"]:
                raise RuntimeError("Simulator stopped unexpectedly")
        return 0
    except BlockingIOError:
        print("A launcher already owns this simulator", file=sys.stderr)
        return 1
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"Startup failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if owned_id:
            # The ID prevents cleanup from stopping a replacement container.
            subprocess.run(docker + ["stop", "--time", "20", owned_id], check=False, stdout=subprocess.DEVNULL)
            subprocess.run(docker + ["rm", owned_id], check=False, stdout=subprocess.DEVNULL)
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
