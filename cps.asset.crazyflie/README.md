# Crazyflie Digital twin-based Controller and Webots Simulator

This directory is self-contained.

## Requirements

- Docker Engine with Compose v2, and Python 3 on the host.
- Linux or WSL2. A working X11/WSLg display is needed for the GUI; use `--headless`
  without a display. NVIDIA on native Linux needs its Docker runtime configured.
- Internet access for the first source build. No prebuilt project image, host ROS,
  Java, Maven, tmux or installation from the parent repository is required.

The image builds from pinned ROS 2 Humble / Ubuntu 22.04 and Webots R2025a sources.

## Build and run

From this directory:

```bash
bash controller/cf.PyControl/bin/start_dssim_docker.sh --action build
bash controller/cf.PyControl/bin/start_dssim_docker.sh
```

If no project image exists, the start command also performs the source build.
Without a display:

```bash
bash controller/cf.PyControl/bin/start_dssim_docker.sh --headless
```

Wait for `READY`. Ctrl+C stops and removes the container created by this launcher.
The image is retained for ordinary reuse. No unrelated processes or containers
are stopped. If a port is occupied, stop its owner explicitly before retrying.

| Interface | Default |
|---|---|
| HTTP | cf231: `http://127.0.0.1:5000/status`, through cf238 on 5007 |
| State diagrams | `http://127.0.0.1:8080/webview.html`, through port 8087 |
| Per-drone WebSocket | 8765–8772 |
| ROSBridge | `ws://127.0.0.1:9090` |
| ROS domain | 41 |
| Webots controller port | 1234 |

The Java-compatible `/cf_positions_path` contains a `nav_msgs/Path`, with `cf231`
through `cf238` in each pose's `header.frame_id`. Native `/cfXXX/status`, `/odom`
and `/scan` messages use the same ROS domain and the same installed interfaces.

Basic HTTP commands are `/activate_idle`, `/begin_takeoff`, `/navigate`,
`/navigate/append`, `/stop_controller` and `/begin_landing`. JSON coordinate requests
and signed URL coordinates are supported. HTTP 200 accepts a task; check
`GET /tasks/<task_id>` for actual completion. Stopping cancels the old task and
holds position. The shared core also retains its polynomial APIs, without
bundling trajectory examples or optional wall-following features.

HTTP, WebSocket and ROSBridge listeners bind all interfaces for the existing
local/Windows integration. They are control endpoints without authentication;
use a trusted network and restrict access when exposing the host.

## World

The editable world is
`simulation/ds-crazyflies-ext/crazyflie.wbt`, mounted at
`/ds/crazywebotsworld/worlds/crazyflie.wbt`. Stop and restart after editing it.
The standalone simulator discovers IDs from this file. The bundled FEDT Java
mission specifically expects the original eight IDs.

`controller/cf.PyControl/src/` contains the three control modules.