"""Execute and switch polynomial trajectories using the existing Crazyflie HTTP API."""

import argparse
import csv
import logging
import time
from pathlib import Path

import requests

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
LOG = logging.getLogger(__name__)

ALL_DRONES = {
    f"cf{231 + i}": f"http://localhost:{5000 + i}"
    for i in range(8)
}

DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "multi_trajectory"


def load_trajectory(path):
    pieces = []
    with path.open(newline="") as file:
        reader = csv.reader(file)
        next(reader, None)
        for line_number, row in enumerate(reader, start=2):
            if not row or all(not cell.strip() for cell in row):
                continue
            if len(row) != 33:
                raise ValueError(f"{path}:{line_number}: expected 33 columns, got {len(row)}")
            values = [float(value) for value in row]
            if values[0] <= 0:
                raise ValueError(f"{path}:{line_number}: duration must be positive")
            pieces.append({
                "duration": values[0],
                "poly_x": values[1:9],
                "poly_y": values[9:17],
                "poly_z": values[17:25],
                "poly_yaw": values[25:33],
            })
    if not pieces:
        raise ValueError(f"No trajectory segments found in {path}")
    return pieces


def request(drone, url, method, endpoint, payload=None):
    response = requests.request(
        method, url + endpoint, json=payload, timeout=10
    )
    try:
        body = response.json()
    except ValueError:
        raise RuntimeError(f"{drone} {endpoint}: invalid response: {response.text}")
    if not response.ok:
        raise RuntimeError(f"{drone} {endpoint}: HTTP {response.status_code}: {body}")
    return body


def broadcast(drones, endpoint, payload=None):
    tasks = {}
    errors = {}
    for drone, url in drones.items():
        try:
            result = request(drone, url, "POST", endpoint, payload)
            task_id = result.get("task_id") if endpoint not in (
                "/trajectory/upload", "/activate_idle"
            ) else None
            tasks[drone] = task_id
            LOG.info("%s %s accepted%s", drone, endpoint,
                     f" task={task_id}" if task_id else "")
        except Exception as exc:
            errors[drone] = str(exc)
            LOG.error("%s", exc)
    if errors:
        raise RuntimeError(f"{endpoint} failed for: {errors}")
    return tasks


def wait_tasks(drones, tasks, timeout=45):
    pending = {drone: task_id for drone, task_id in tasks.items() if task_id}
    deadline = time.monotonic() + timeout
    while pending:
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Tasks did not finish within {timeout}s: {pending}")
        for drone, task_id in list(pending.items()):
            result = request(drone, drones[drone], "GET", f"/tasks/{task_id}")
            status = result.get("status")
            if status == "completed":
                LOG.info("%s task %s completed", drone, task_id)
                del pending[drone]
            elif status in ("failed", "cancelled"):
                raise RuntimeError(
                    f"{drone} task {task_id}: {status}: {result.get('error')}"
                )
            elif status not in ("queued", "running"):
                raise RuntimeError(f"{drone} task {task_id}: unknown status {status}")
        if pending:
            time.sleep(0.2)


def check_ready(drones):
    for drone, url in drones.items():
        result = request(drone, url, "GET", "/status")
        if not result.get("connected") or not result.get("services_ready"):
            raise RuntimeError(f"{drone} is not ready: {result}")
        LOG.info("%s ready: phase=%s position=%s",
                 drone, result["phase"], result["telemetry"]["position"])


def mission(drones, interrupt_after):
    pieces0 = load_trajectory(DATA_DIR / "traj0.csv")
    pieces1 = load_trajectory(DATA_DIR / "traj1.csv")

    check_ready(drones)

    # Begin from a fresh simulation state; restart the simulator between tests.
    broadcast(drones, "/activate_idle")
    takeoff = broadcast(drones, "/begin_takeoff", {"height": 1.0, "duration": 3.0})
    wait_tasks(drones, takeoff)

    broadcast(drones, "/trajectory/upload", {
        "trajectory_id": 0,
        "pieces": pieces0,
        "upload_to_vehicle": False,
    })
    first = broadcast(drones, "/trajectory/full_state/start", {
        "trajectory_id": 0, "timescale": 1.0, "relative": True,
    })

    LOG.info("Trajectory 0 running; interrupting after %.1f seconds", interrupt_after)
    time.sleep(interrupt_after)

    # A cancelled trajectory task is expected when /trajectory/stop is called.
    stopped = broadcast(drones, "/trajectory/stop")
    wait_tasks(drones, stopped)

    for drone, task_id in first.items():
        result = request(drone, drones[drone], "GET", f"/tasks/{task_id}")
        LOG.info("%s interrupted trajectory status=%s", drone, result["status"])
        if result["status"] != "cancelled":
            raise RuntimeError(
                f"{drone}: expected trajectory 0 to be cancelled, got {result['status']}"
            )

    before_second = {
        drone: request(drone, url, "GET", "/status")["telemetry"]["position"]
        for drone, url in drones.items()
    }

    broadcast(drones, "/trajectory/upload", {
        "trajectory_id": 1,
        "pieces": pieces1,
        "upload_to_vehicle": False,
    })
    second = broadcast(drones, "/trajectory/full_state/start", {
        "trajectory_id": 1, "timescale": 1.0, "relative": True,
    })
    wait_tasks(drones, second)

    for drone, url in drones.items():
        status = request(drone, url, "GET", "/status")
        final_position = status["telemetry"]["position"]
        displacement_y = final_position[1] - before_second[drone][1]
        LOG.info(
            "%s after trajectory 1: position=%s, Y displacement=%.3f m",
            drone, final_position, displacement_y
        )
        if not 0.20 <= displacement_y <= 0.40:
            raise RuntimeError(
                f"{drone}: unexpected Y displacement {displacement_y:.3f} m"
            )

    landing = broadcast(drones, "/begin_landing", {"duration": 3.0})
    wait_tasks(drones, landing)
    LOG.info("MISSION COMPLETED SUCCESSFULLY")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drones", nargs="+", default=["cf231"],
                        choices=sorted(ALL_DRONES))
    parser.add_argument("--interrupt-after", type=float, default=2.0)
    args = parser.parse_args()

    duration = sum(p["duration"] for p in load_trajectory(DATA_DIR / "traj0.csv"))
    if not 0 < args.interrupt_after < duration:
        parser.error(f"--interrupt-after must be between 0 and {duration} seconds")

    drones = {name: ALL_DRONES[name] for name in args.drones}
    try:
        mission(drones, args.interrupt_after)
    except Exception:
        LOG.exception("MISSION FAILED")

        # Best-effort recovery: request controlled braking/hovering.
        # This is not an immediate motor shutdown.
        for drone, url in drones.items():
            try:
                status = request(drone, url, "GET", "/status")
                if status.get("phase") in (
                    "hovering", "full_state", "trajectory",
                    "navigating", "braking", "taking_off"
                ):
                    result = request(drone, url, "POST", "/trajectory/stop")
                    LOG.warning(
                        "%s recovery stop requested: task=%s",
                        drone, result.get("task_id")
                    )
            except Exception as recovery_error:
                LOG.error(
                    "%s recovery stop failed: %s",
                    drone, recovery_error
                )

        raise SystemExit(1)


if __name__ == "__main__":
    main()
