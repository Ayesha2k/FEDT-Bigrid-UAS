# Multi-Drone Trajectory Switching

## Objective
Reproduce Crazyswarm2 multi-trajectory behavior using the existing HTTP API:
take off, fly a trajectory, interrupt it, brake, fly another trajectory, land.

## Files
- `src/multi_trajectory_client.py`: HTTP mission client.
- `data/multi_trajectory/traj0.csv`: 6-second, +0.3 m X trajectory.
- `data/multi_trajectory/traj1.csv`: 6-second, +0.3 m Y trajectory.

Paths are relative to `controller/cf.PyControl`.

## Run
From the `cps.asset.crazyflie` directory, start the simulator:

    bash controller/cf.PyControl/bin/start_dssim_docker.sh webots --headless

In another terminal, run:

    python3 controller/cf.PyControl/src/multi_trajectory_client.py --drones cf231 cf233 cf235 --interrupt-after 2.0

The drone HTTP interfaces use localhost ports 5000-5007.

## Implementation
The client reuses `/trajectory/upload`, `/trajectory/full_state/start`,
`/trajectory/stop`, and `/tasks/<id>` instead of introducing new endpoints.
It verifies cancellation, task completion, and final Y displacement.

## Problems and Solutions
1. Zero-valued trajectories: replaced with smooth, nonzero polynomials.
2. Redundant HTTP endpoints: reused existing trajectory start/stop APIs.
3. Unverified interruption: wait for braking and confirm task cancellation.
4. Misleading upload task IDs: log IDs only for asynchronous operations.
5. Headless startup issues: corrected display handling and removed the
   unavailable wall-following webview service.
6. Missing movement checks: validate +0.3 m Y displacement (tolerance 0.1 m).

## Test Results
15 November 2026: cf231, cf233, and cf235 completed takeoff,
trajectory interruption after 2 seconds, braking, second trajectory,
and landing. Each achieved approximately +0.300 m Y displacement.

Result: MISSION COMPLETED SUCCESSFULLY.
The mission also passed when repeated without restarting Docker.

## Known Limitations
- Observed altitude after Trajectory 1: approximately 0.875-0.876 m,
  compared with the nominal 1.0 m takeoff target; cause unresolved.
- Failure recovery requests controlled braking, not motor shutdown.
  Recovery during an actual failure has not been tested.
- Displacement validation assumes the current +0.3 m Y trajectory.
- HTTP requests are sequential, not perfectly synchronized.
