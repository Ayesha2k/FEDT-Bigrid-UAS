# FEDT control runtime

The HTTP server calls `DroneController` directly. Each drone owns its task queue,
telemetry and `DroneState`; PNG publication runs asynchronously. This bundle
contains the runtime only, without radio clients, wall-following, examples or
analysis scripts.

Build, launch and interface instructions are in [the standalone simulator guide](../../README.md).
No Java application or grid provider is needed for this controller to run.
