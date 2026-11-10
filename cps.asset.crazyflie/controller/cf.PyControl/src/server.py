#!/usr/bin/env python3
"""Flat HTTP entry point. Existing URLs remain per-drone; new tasks are additive."""
import argparse
import asyncio
import json
import logging
import math
from pathlib import Path
import signal
import threading

from flask import Flask, jsonify, request
from werkzeug.exceptions import HTTPException
from werkzeug.routing import BaseConverter

from drone_controller import DroneController, RosTransport, ControlError, finite, positive


class SignedNumber(BaseConverter):
    regex = r"-?\d+(?:\.\d+)?"
    def to_python(self, value):
        return float(value)


def create_app(controller, web_root=None):
    root = Path(web_root or Path(__file__).resolve().parents[1] / "webview")
    app = Flask(__name__, static_folder=str(root), static_url_path="")
    app.url_map.converters["number"] = SignedNumber
    app.config["MAX_CONTENT_LENGTH"] = 4 * 1024 * 1024
    app.config["CONTROLLER"] = controller

    def data():
        value = request.get_json(silent=True)
        if value is None:
            if request.data and request.data.strip() not in (b"", b"{}"):
                raise ValueError("Expected a JSON object")
            return {}
        if not isinstance(value, dict):
            raise ValueError("Expected a JSON object")
        return value

    def coordinates(payload):
        return [finite(payload[k], k) for k in ("x", "y", "z")]

    def reply(message, task_id=None, **extra):
        result = dict(controller.snapshot(), message=message, **extra)
        if task_id:
            result["task_id"] = task_id
        return jsonify(result)

    @app.errorhandler(Exception)
    def error(exc):
        if isinstance(exc, HTTPException):
            return jsonify(error=exc.description), exc.code
        if isinstance(exc, ControlError):
            return jsonify(error=str(exc)), exc.status
        if isinstance(exc, (ValueError, KeyError, TypeError)):
            return jsonify(error=str(exc)), 400
        logging.exception("Request failed")
        return jsonify(error=str(exc)), 503

    @app.get("/status")
    @app.get("/state")
    def status():
        return jsonify(controller.snapshot())

    @app.get("/transition")
    def transition():
        return jsonify(transition=controller.snapshot()["transition"])

    @app.get("/routes")
    def routes():
        return jsonify(routes=[request.host_url.rstrip("/") + str(r) for r in app.url_map.iter_rules()])

    @app.get("/capabilities")
    def capabilities():
        return jsonify(drone_id=controller.drone_id, backend="ros2",
                       webots_motion_limits={"linear_m_s": 3.0, "angular_rad_s": 6.0} if getattr(controller.transport, "require_report", False) else None,
                       task_status=["queued", "running", "completed", "failed", "cancelled"],
                       trajectories="piecewise polynomial, constant coefficient first",
                       dynamics="simplified Webots motion; not firmware or motor-level equivalence")

    @app.post("/activate_idle")
    def idle():
        controller.activate_idle()
        return reply("Activated idle")

    @app.post("/begin_takeoff")
    def takeoff():
        p = data()
        task = controller.takeoff(p.get("height", 0.5), p.get("duration", 2.0))
        return reply("Takeoff initiated", task)

    @app.post("/begin_landing")
    def land():
        p = data()
        task = controller.land(p.get("height", 0.0), p.get("duration", 2.0))
        return reply("Landing initiated", task)

    @app.post("/navigate")
    @app.post("/navigate/append", defaults={"append": True})
    def navigate(append=False):
        p = data()
        task = controller.navigate(coordinates(p), append, p.get("duration", 10.0))
        return reply("Navigation point added", task)

    @app.post("/navigate/<number:x>/<number:y>/<number:z>")
    @app.post("/navigate/append/<number:x>/<number:y>/<number:z>", defaults={"append": True})
    def navigate_path(x, y, z, append=False):
        task = controller.navigate([x, y, z], append)
        return reply("Navigation point added", task)

    @app.post("/hover/realtime")
    def hover():
        p = data()
        supplied = [k in p for k in ("x", "y", "z")]
        if any(supplied) and not all(supplied):
            raise ValueError("Provide all of x/y/z or no coordinates")
        controller.hover(coordinates(p) if all(supplied) else None, p.get("yaw", 0))
        return reply("Real-time hover")

    @app.post("/move/realtime")
    def move():
        p = data()
        task = controller.move(coordinates(p), p.get("velocity", 0.1), p.get("timeout", 60))
        return reply("Real-time move initiated", task)

    @app.post("/stop_controller")
    @app.post("/trajectory/stop")
    def stop():
        controller.hover()
        return reply("Controller stopped; holding position")

    @app.post("/trajectory/upload")
    def upload():
        p = data()
        controller.upload(p.get("trajectory_id", 0), p["pieces"], p.get("piece_offset", 0))
        return reply("Trajectory uploaded", trajectory_id=int(p.get("trajectory_id", 0)))

    @app.post("/trajectory/start")
    @app.post("/trajectory/full_state/start", defaults={"full_state": True})
    def start_trajectory(full_state=False):
        p = data()
        task = controller.start_trajectory(p.get("trajectory_id", 0), p.get("timescale", 1),
            p.get("reversed", False), p.get("relative", True), full_state, p.get("rate", 30))
        return reply("Trajectory initiated", task)

    @app.get("/tasks/<task_id>")
    def task(task_id):
        return jsonify(controller.task(task_id))

    def lifecycle(event):
        controller.lifecycle(event)
        return reply("Transitioned")
    for event in ("install", "start", "initialize", "stop", "uninstall", "shutdown_command", "begin_stopping"):
        app.add_url_rule("/" + event, "lifecycle_" + event, lambda event=event: lifecycle(event), methods=["POST"])
    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--drone_id", default="cf231")
    clock = parser.add_mutually_exclusive_group()
    clock.add_argument("--use-sim-time", dest="use_sim_time", action="store_true")
    clock.add_argument("--use-wall-time", dest="use_sim_time", action="store_false")
    parser.set_defaults(use_sim_time=False)
    parser.add_argument("--require-command-report", action="store_true", help="Validate ds3 execution diagnostics, not just empty ROS responses")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--wsendpoint", action="store_true")
    parser.add_argument("--wshost", default="0.0.0.0")
    parser.add_argument("--wsport", type=int, default=8765)
    parser.add_argument("--web-root", type=Path, default=Path(__file__).resolve().parents[1] / "webview")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO)
    import rclpy
    from waitress import create_server
    rclpy.init()
    controller = DroneController(args.drone_id, RosTransport(args.drone_id, use_sim_time=args.use_sim_time, require_report=args.require_command_report),
                                 image_dir=args.web_root / "img")
    http = None
    stopped = threading.Event()
    def shutdown(*_):
        stopped.set()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        http = create_server(create_app(controller, args.web_root), host=args.host, port=args.port, threads=8)
        threading.Thread(target=http.run, daemon=True).start()
        if args.wsendpoint:
            async def websocket_main():
                import websockets
                async def stream(ws, *unused):
                    while not stopped.is_set():
                        s = controller.snapshot()
                        t = s["telemetry"] or {}
                        p = t.get("position", [0, 0, 0])
                        x, y, z, w = t.get("orientation", [0, 0, 0, 1])
                        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
                        pitch = math.asin(max(-1, min(1, 2*(w*y-z*x))))
                        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                        v = t.get("velocity", [0, 0, 0])
                        value = dict(x=p[0], y=p[1], z=p[2], roll=roll, pitch=pitch, yaw=yaw,
                                     battery=t.get("battery", 0), batteryLevel=0, batteryState=t.get("pm_state", 0),
                                     acc_x=v[0], acc_y=v[1], acc_z=v[2])
                        await ws.send(json.dumps({"message": "crazyflie_position", "value": {args.drone_id: value}}))
                        await asyncio.sleep(1)
                async with websockets.serve(stream, args.wshost, args.wsport):
                    while not stopped.is_set():
                        await asyncio.sleep(0.1)
            threading.Thread(target=lambda: asyncio.run(websocket_main()), daemon=True).start()
        stopped.wait()
    finally:
        if http:
            http.close()
        controller.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
