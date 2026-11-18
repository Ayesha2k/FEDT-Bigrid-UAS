# Task 3 — 3D CBS for Crazyflie

## Implementation
Standalone Python 3D grid-based CBS prototype with collision and downwash-aware planning.
The downwash check uses a 0.6 m vertical threshold and a configurable horizontal radius.

- collision_checker.py: 3D collision and downwash checks.
- planner_3d.py: 3D CBS path planning.
- server.py: HTTP planning service (GET /health, POST /plan).
- trajectory_converter.py: Polynomial trajectory generation and validation.
- prepare_flight.py: Integration with Crazyflie HTTP controllers.

## Usage
Start the planner with: python3 server.py
With the Crazyflie simulation running, use: python3 prepare_flight.py
The flight preparation script requires the Python requests package.

## Validation
Tested HTTP planning, trajectory conversion, safety checks, and two-drone Webots execution.
Both drones completed their trajectories and landed successfully.

## Limitations
This is a standalone grid-based CBS prototype, not yet an extension of the original C++ kinodynamic DB-CBS.
Integrating 3D robot dynamics and motion primitives into the original algorithm is the next step.
