<div align="center">

# FEDT-Bigraph-UAS

[![Project Page](https://img.shields.io/badge/🌐_Project_Page-1f6feb?style=flat-square)](https://shanecan.github.io/FEDT-Bigrid-UAS/)
[![Video](https://img.shields.io/badge/📺_Video-FF0000?style=flat-square)](https://www.youtube.com/watch?v=338XnNnXHFI)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg?style=flat-square)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square&logo=ros)](https://docs.ros.org/)
[![BigraphER](https://img.shields.io/badge/BigraphER-2.0.0-5b21b6?style=flat-square)](https://www.dcs.gla.ac.uk/~michele/bigrapher.html)
[![PRISM](https://img.shields.io/badge/PRISM-4.9-0e7490?style=flat-square)](https://www.prismmodelchecker.org/)

</div>

<p align="center">
  <img src="docs/static/images/framework.png" alt="FEDT framework architecture" width="100%">
</p>

<p align="center"><em>
Formal Model-Based Executable Digital Twin (FEDT) framework architecture. The pipeline couples physical UAS execution, discrete bigraph-based
reasoning, runtime validation, and monitoring in a closed loop.
</em></p>

## Intro

This project demonstrates how Bigraphs can be used to model and control Crazyflie2.1 drones in simulation or reality. It implements a grid-based, collision-free UAV navigation system using Bigraphical Reactive Systems (BRS), ROS2, and A* path planning. This allows UAVs to fly safely from takeoff to landing in a discretized airspace while maintaining real-time digital twin consistency between the world model, UAV agent model, and live ROS2 data.

<table>
<tr>
<td align="center" valign="middle"><img src="docs/static/images/demo-twin.gif" alt="Bi-grid digital twin: live grid occupancy and UAS state" height="260"></td>
<td align="center" valign="middle"><img src="docs/static/images/demo-flight.gif" alt="Indoor Crazyflie testbed during a multi-UAS run" height="260"></td>
</tr>
<tr>
<td align="center"><em>Digital twin — live grid occupancy and UAS state</em></td>
<td align="center"><em>Physical layer — indoor Crazyflie testbed</em></td>
</tr>
</table>

## Contributions

1. **A unified bigraph modelling method** for UAS flight operations in discretised 3D airspace,
   integrating airspace topology, occupancy, adjacency, mission phase, battery state, and
   communication state into a single executable runtime representation.

2. **A pre-execution verification and fail-safe mechanism** based on bigraph rewriting. Candidate
   movements and state transitions are executed only when their reaction-rule preconditions hold,
   enforcing grid occupancy exclusion and logical consistency before actuation.

3. **An end-to-end runtime assurance system** with ROS-based UAS execution, model synchronisation,
   and operator monitoring.

The framework is **planner-independent**: it acts as a runtime assurance layer that supervises the
outputs of existing planners or controllers, rather than serving as a trajectory generation or
formation control algorithm.

> Across nine scenarios and 1813 runtime-gate decisions, FEDT recorded **zero grid co-occupancy
> violations**. Full results are on the
> [project page](https://shanecan.github.io/FEDT-Bigrid-UAS/).

## Repository layout

| Path | Role |
| --- | --- |
| [`bigrid-provider-service/`](bigrid-provider-service/) | Spring Boot REST API that generates and serves the *bi-grid* — the bigraph world model with grid topology (Ecore XMI / JSON / Protobuf). |
| [`cps.asset.crazyflie/`](cps.asset.crazyflie/) | Self-contained Webots simulator, current Python HTTP control core, native ROSBridge and state diagrams.|
| [`bigraphs.drone-collision-free/`](bigraphs.drone-collision-free/) | The FEDT runtime: joint bigraph world + UAS model, A\* path planning, the runtime gate, and CDO model persistence. |
| [`bigrid/`](bigrid/) | BigraphER + PRISM model checking of the reaction-rule set. See [`bigrid/README.md`](bigrid/README.md). |

## Installation

Everything lives in this single repository.

```bash
git clone https://github.com/ShaneCan/FEDT-Bigrid-UAS.git
cd FEDT-Bigrid-UAS
```

### Runtime requirements

The simulator container is pinned to **Ubuntu 22.04 / ROS 2 Humble / Webots R2025a**.
The host needs Docker Engine, Compose v2 and Python 3. A GUI requires X11 or WSLg;
headless operation uses a private virtual display. Host ROS is not required.

For the **full FEDT system**, also install **JDK 21** and **tmux**. Set `JAVA_HOME`
to the intended JDK if the host's default Java is older. The launchers do not
install or upgrade host Java, ROS or drivers. Maven is provided by each Java
module's wrapper.

### Build from source

From the repository root:

```bash
bash cps.asset.crazyflie/controller/cf.PyControl/bin/start_dssim_docker.sh --action build
(cd bigraphs.drone-collision-free && ./mvnw clean package -DskipTests)
(cd bigrid-provider-service && ./mvnw clean package -DskipTests)
```

The first build downloads the pinned dependencies. World assets are bundled
locally.

## Usage

### Full FEDT system

```bash
bash launch_application.sh
```

The private tmux session shows four logs: simulator, CDO, grid provider and Java
application. The launcher waits for each dependency. Press **Ctrl+Q** in tmux to stop this session and the services it created. CDO uses port 2036, the grid provider uses 7070, and the application is pointed at `http://127.0.0.1:7070`. Fault injection remains on 8090, outside the eight state view ports.

```bash
bash launch_application.sh --headless
```

No GUI mode.

### Simulator only

```bash
bash launch_application.sh --sim-only
# Without a Webots window:
bash launch_application.sh --sim-only --headless
```

This mode does not require Java, Maven, tmux, CDO or the grid service. You can run
your own HTTP/ROSBridge client, including a Java application on Windows, after
`READY`. Do not let two application clients command the same drones concurrently.

For a simulator-only copy, retain only `cps.asset.crazyflie/`; the other FEDT
modules can be removed. From the repository root:

```bash
cd cps.asset.crazyflie
bash controller/cf.PyControl/bin/start_dssim_docker.sh
```

See [the standalone guide](cps.asset.crazyflie/README.md) for endpoints, world
editing, source builds and GPU settings.

## Model checking

The reaction-rule set is verified independently of the running system. The bigraphical reactive
system is exported as a DTMC with **BigraphER** and checked with **PRISM**, confirming that no two
UAS ever occupy the same grid cell and that no landed UAS is left at an air locale.

- Install BigraphER via opam following the
  [official instructions](https://www.dcs.gla.ac.uk/~michele/bigrapher.html#opam).
- Install [PRISM](https://www.prismmodelchecker.org/download.php).

```bash
cd bigrid
bash run_verified_analysis.sh        # exhaustive verification of the 2x2x2 configuration
bash verification_analysis.sh 2000   # bounded exploration of the 3x3x3 configuration
```

See [`bigrid/README.md`](bigrid/README.md) for the models, the properties that are checked, and the
expected output.

## Citation

```bibtex
@article{zhang2026fedt,
  title  = {Runtime-Assured Digital Twins for Safe UAS Operations in Gridded Airspace},
  author = {Zhang, Tianxiong and Grzelak, Dominik and Victor, Victor and
            Sevegnani, Michele and Fricke, Hartmut and A{\ss}mann, Uwe},
  year   = {2026}
}
```

## License

Project code is released under the [MIT License](LICENSE). Bundled upstream runtime and world resources retain their own licenses and attribution.
