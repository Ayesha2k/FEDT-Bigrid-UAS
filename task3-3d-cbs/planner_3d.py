"""3D grid-based Conflict-Based Search for multiple drones.

This is a discrete CBS implementation, not the original kinodynamic
DB-CBS algorithm. Paths require smoothing and dynamic validation
before execution on physical or simulated drones.
"""

import heapq
import itertools
import math

from collision_checker import SafetyConfig, check_conflict


MOVES = [
    (0, 0, 0),
    (1, 0, 0), (-1, 0, 0),
    (0, 1, 0), (0, -1, 0),
    (0, 0, 1), (0, 0, -1),
]


def distance(a, b):
    return sum(abs(a[i] - b[i]) for i in range(3))


def position_at(path, t):
    return path[min(t, len(path) - 1)]


def to_world(cell, resolution):
    return tuple(round(v * resolution, 6) for v in cell)


def find_conflict(paths, config, resolution):
    """Find earliest vertex, downwash, or edge-swap conflict."""

    horizon = max(len(path) for path in paths)

    for t in range(horizon):
        for i, j in itertools.combinations(range(len(paths)), 2):
            a = position_at(paths[i], t)
            b = position_at(paths[j], t)

            conflict = check_conflict(
                to_world(a, resolution),
                to_world(b, resolution),
                config,
            )

            if conflict:
                return {
                    "agents": (i, j),
                    "time": t,
                    "type": conflict["type"],
                }

            if t > 0:
                previous_a = position_at(paths[i], t - 1)
                previous_b = position_at(paths[j], t - 1)

                if previous_a == b and previous_b == a:
                    return {
                        "agents": (i, j),
                        "time": t,
                        "type": "edge_swap",
                    }

                # Detect proximity during simultaneous movement.
                # The distance minimum along two linear segments occurs
                # at the clamped projection of relative displacement.
                r0 = [
                    (previous_a[k] - previous_b[k]) * resolution
                    for k in range(3)
                ]
                rv = [
                    ((a[k] - previous_a[k]) -
                     (b[k] - previous_b[k])) * resolution
                    for k in range(3)
                ]

                denom = sum(v * v for v in rv)
                fraction = (
                    max(0.0, min(1.0, -sum(r0[k] * rv[k]
                    for k in range(3)) / denom))
                    if denom > 1e-12 else 0.0
                )

                interpolated_a = [
                    (previous_a[k] +
                     fraction * (a[k] - previous_a[k])) * resolution
                    for k in range(3)
                ]
                interpolated_b = [
                    (previous_b[k] +
                     fraction * (b[k] - previous_b[k])) * resolution
                    for k in range(3)
                ]

                conflict = check_conflict(
                    interpolated_a, interpolated_b, config
                )

                if conflict:
                    return {
                        "agents": (i, j),
                        "time": t,
                        "type": "transition_" + conflict["type"],
                    }

    return None


def astar(start, goal, bounds, constraints, agent,
          max_time=80):
    """Time-expanded 3D A* with vertex and transition constraints."""

    counter = itertools.count()
    queue = [
        (distance(start, goal), 0, next(counter), start, [start])
    ]

    visited = set()

    last_constraint = max(
        (c[1] for c in constraints if c[0] == agent),
        default=0,
    )

    while queue:
        _, t, _, current, path = heapq.heappop(queue)

        if (current, t) in visited:
            continue

        visited.add((current, t))

        if current == goal and t > last_constraint:
            return path

        if t >= max_time:
            continue

        for move in MOVES:
            nxt = tuple(current[k] + move[k] for k in range(3))

            if not all(
                bounds[k][0] <= nxt[k] <= bounds[k][1]
                for k in range(3)
            ):
                continue

            new_time = t + 1

            if (agent, new_time, nxt) in constraints:
                continue

            if (agent, new_time, current, nxt) in constraints:
                continue

            if (nxt, new_time) in visited:
                continue

            new_path = path + [nxt]

            heapq.heappush(
                queue,
                (
                    new_time + distance(nxt, goal),
                    new_time,
                    next(counter),
                    nxt,
                    new_path,
                ),
            )

    return None


def plan(starts, goals, bounds, resolution=0.5,
         config=None, max_time=80, max_nodes=3000):
    """Plan conflict-free 3D grid paths using CBS."""

    if config is None:
        config = SafetyConfig()

    if len(starts) != len(goals) or not starts:
        raise ValueError("Starts and goals must have equal nonzero length")

    if resolution <= 0:
        raise ValueError("Resolution must be positive")

    if len(bounds) != 3:
        raise ValueError("Bounds must contain X, Y, Z limits")

    def grid(point):
        if len(point) != 3:
            raise ValueError("Each position must have 3 coordinates")
        return tuple(round(float(v) / resolution) for v in point)

    grid_starts = [grid(p) for p in starts]
    grid_goals = [grid(p) for p in goals]

    grid_bounds = [
        (
            math.ceil(float(lo) / resolution),
            math.floor(float(hi) / resolution),
        )
        for lo, hi in bounds
    ]

    for point in grid_starts + grid_goals:
        if not all(
            grid_bounds[k][0] <= point[k] <= grid_bounds[k][1]
            for k in range(3)
        ):
            raise ValueError("Start or goal outside planning bounds")

    constraints = frozenset()

    paths = []

    for i, (start, goal) in enumerate(
        zip(grid_starts, grid_goals)
    ):
        path = astar(
            start, goal, grid_bounds, constraints, i, max_time
        )
        if path is None:
            return None
        paths.append(path)

    counter = itertools.count()
    queue = [
        (sum(map(len, paths)), next(counter), constraints, paths)
    ]

    explored = set()
    expanded = 0

    while queue and expanded < max_nodes:
        _, _, constraints, paths = heapq.heappop(queue)

        if constraints in explored:
            continue

        explored.add(constraints)
        expanded += 1

        conflict = find_conflict(paths, config, resolution)

        if conflict is None:
            return {
                "paths": [
                    [list(to_world(p, resolution)) for p in path]
                    for path in paths
                ],
                "expanded_nodes": expanded,
                "resolution": resolution,
                "algorithm": "3D grid-based CBS",
            }

        i, j = conflict["agents"]
        t = conflict["time"]

        for agent in (i, j):
            new_constraints = set(constraints)

            if conflict["type"].startswith("transition_") or (
                conflict["type"] == "edge_swap"
            ):
                previous = position_at(paths[agent], t - 1)
                current = position_at(paths[agent], t)

                new_constraints.add(
                    (agent, t, previous, current)
                )
            else:
                new_constraints.add(
                    (agent, t, position_at(paths[agent], t))
                )

            new_constraints = frozenset(new_constraints)

            new_path = astar(
                grid_starts[agent],
                grid_goals[agent],
                grid_bounds,
                new_constraints,
                agent,
                max_time,
            )

            if new_path is None:
                continue

            new_paths = list(paths)
            new_paths[agent] = new_path

            heapq.heappush(
                queue,
                (
                    sum(map(len, new_paths)),
                    next(counter),
                    new_constraints,
                    new_paths,
                ),
            )

    return None


if __name__ == "__main__":
    result = plan(
        starts=[
            [-1, 0, 1],
            [1, 0, 1],
        ],
        goals=[
            [1, 0, 1],
            [-1, 0, 1],
        ],
        bounds=[
            [-2, 2],
            [-2, 2],
            [0.5, 2],
        ],
        resolution=0.5,
    )

    if result is None:
        print("No solution found")
    else:
        print("Algorithm:", result["algorithm"])
        print("Expanded nodes:", result["expanded_nodes"])

        for i, path in enumerate(result["paths"]):
            print(f"Drone {i}: {path}")

        grid_paths = [
            [tuple(round(v / 0.5) for v in p) for p in path]
            for path in result["paths"]
        ]

        assert find_conflict(
            grid_paths, SafetyConfig(), 0.5
        ) is None

        print("SUCCESS: No detected conflicts")
