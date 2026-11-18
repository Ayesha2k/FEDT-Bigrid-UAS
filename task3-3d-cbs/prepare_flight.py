"""Prepare 3D CBS trajectories for Crazyflie without commanding flight."""

import json
import math
import sys
from pathlib import Path

import requests

from trajectory_converter import (
    convert_paths,
    validate_trajectories,
    save_csv,
)

DRONES = {
    "cf231": "http://127.0.0.1:5000",
    "cf232": "http://127.0.0.1:5001",
}

TAKEOFF_HEIGHT = 1.0
RESOLUTION = 0.1
SEGMENT_DURATION = 3.0

OUTPUT = Path(__file__).resolve().parent / "generated" / "live"


def main():
    starts = []

    for name, url in DRONES.items():
        response = requests.get(f"{url}/status", timeout=5)
        response.raise_for_status()

        status = response.json()

        if status.get("phase") != "landed":
            raise RuntimeError(
                f"{name} must be landed for this preparation test"
            )

        position = status["telemetry"]["position"]

        starts.append([
            position[0],
            position[1],
            TAKEOFF_HEIGHT,
        ])

    goals = [
        [starts[0][0] + 0.2, starts[0][1], TAKEOFF_HEIGHT],
        [starts[1][0] - 0.2, starts[1][1], TAKEOFF_HEIGHT],
    ]

    payload = {
        "starts": starts,
        "goals": goals,
        "bounds": [
            [-2.0, 2.0],
            [-2.0, 2.5],
            [0.5, 1.5],
        ],
        "resolution": RESOLUTION,
        "max_time": 40,
        "max_nodes": 3000,
    }

    response = requests.post(
        "http://127.0.0.1:5050/plan",
        json=payload,
        timeout=30,
    )
    response.raise_for_status()

    result = response.json()

    if result["status"] != "success":
        raise RuntimeError("CBS planning failed")

    trajectories = convert_paths(
        result["paths"],
        segment_duration=SEGMENT_DURATION,
    )

    # Translate each drone's complete path to remove grid rounding error.
    for start, trajectory, path in zip(
        starts, trajectories, result["paths"]
    ):
        offset = [
            start[i] - path[0][i]
            for i in range(3)
        ]

        for segment in trajectory:
            for i, axis in enumerate(("x", "y", "z")):
                segment[f"poly_{axis}"][0] += offset[i]

    # Safety must be checked AFTER applying individual offsets.
    validate_trajectories(
        trajectories,
        sample_interval=0.02,
    )

    OUTPUT.mkdir(parents=True, exist_ok=True)

    for (name, _), trajectory in zip(
        DRONES.items(), trajectories
    ):
        filename = OUTPUT / f"{name}.csv"
        save_csv(trajectory, filename)

        print(
            f"{name}: {len(trajectory)} segments, "
            f"{sum(s['duration'] for s in trajectory):.1f}s"
        )
        print("Saved:", filename)

    print("\nSUCCESS: Live-position trajectories prepared and validated")
    print("No flight commands were sent.")


if __name__ == "__main__":
    main()
