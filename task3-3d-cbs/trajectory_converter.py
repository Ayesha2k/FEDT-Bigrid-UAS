"""Convert time-indexed 3D CBS paths into Crazyflie polynomial trajectories."""

import csv
import math
from pathlib import Path

from collision_checker import SafetyConfig, check_conflict


HEADER = (
    ["duration"]
    + [f"p{axis}{i}" for axis in ("x", "y", "z", "yaw") for i in range(8)]
)


def smooth_segment(start, end, duration):
    """Quintic polynomial with zero endpoint velocity and acceleration."""
    displacement = end - start
    T = duration

    return [
        start,
        0.0,
        0.0,
        10.0 * displacement / T**3,
        -15.0 * displacement / T**4,
        6.0 * displacement / T**5,
        0.0,
        0.0,
    ]


def evaluate(coefficients, t):
    return sum(c * t**i for i, c in enumerate(coefficients))


def convert_paths(paths, segment_duration=3.0):
    if segment_duration <= 0:
        raise ValueError("segment_duration must be positive")

    if not paths or any(not path for path in paths):
        raise ValueError("All drones must have nonempty paths")

    horizon = max(len(path) for path in paths)

    # Preserve the planner's synchronized discrete time steps.
    padded = [
        path + [path[-1]] * (horizon - len(path))
        for path in paths
    ]

    trajectories = []

    for path in padded:
        segments = []

        for i in range(horizon - 1):
            start = path[i]
            end = path[i + 1]

            if len(start) != 3 or len(end) != 3:
                raise ValueError("Each waypoint must contain x, y, z")

            segment = {
                "duration": segment_duration,
                "poly_x": smooth_segment(start[0], end[0], segment_duration),
                "poly_y": smooth_segment(start[1], end[1], segment_duration),
                "poly_z": smooth_segment(start[2], end[2], segment_duration),
                "poly_yaw": [0.0] * 8,
            }

            segments.append(segment)

        trajectories.append(segments)

    return trajectories


def position_at(segments, time):
    if not segments:
        raise ValueError("Trajectory has no segments")

    remaining = time

    for segment in segments:
        duration = segment["duration"]

        if remaining <= duration:
            return [
                evaluate(segment[f"poly_{axis}"], remaining)
                for axis in ("x", "y", "z")
            ]

        remaining -= duration

    last = segments[-1]

    return [
        evaluate(last[f"poly_{axis}"], last["duration"])
        for axis in ("x", "y", "z")
    ]


def validate_trajectories(
    trajectories,
    config=None,
    sample_interval=0.05,
):
    """Sample simultaneous trajectories for collisions/downwash."""

    if config is None:
        config = SafetyConfig()

    if sample_interval <= 0:
        raise ValueError("sample_interval must be positive")

    durations = [
        sum(segment["duration"] for segment in trajectory)
        for trajectory in trajectories
    ]

    if not durations:
        raise ValueError("No trajectories supplied")

    if max(durations) - min(durations) > 1e-6:
        raise ValueError("Trajectories must have equal durations")

    duration = durations[0]
    samples = math.ceil(duration / sample_interval)

    for step in range(samples + 1):
        t = min(step * sample_interval, duration)

        positions = [
            position_at(trajectory, t)
            for trajectory in trajectories
        ]

        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                conflict = check_conflict(
                    positions[i],
                    positions[j],
                    config,
                )

                if conflict:
                    raise ValueError(
                        f"Conflict at t={t:.2f}s between "
                        f"drones {i} and {j}: {conflict}"
                    )

    return True


def save_csv(segments, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(HEADER)

        for segment in segments:
            row = [segment["duration"]]

            for axis in ("x", "y", "z", "yaw"):
                row.extend(segment[f"poly_{axis}"])

            writer.writerow(row)

    return path
