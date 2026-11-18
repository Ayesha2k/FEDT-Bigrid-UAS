"""HTTP API for the 3D CBS prototype."""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from collision_checker import SafetyConfig
from planner_3d import plan


class PlannerHandler(BaseHTTPRequestHandler):

    def send_json(self, status, data):
        body = json.dumps(data).encode("utf-8")

        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self.send_json(200, {
                "status": "ok",
                "service": "crazyflie-3d-cbs",
            })
        else:
            self.send_json(404, {"error": "Unknown endpoint"})

    def do_POST(self):
        if self.path != "/plan":
            self.send_json(404, {"error": "Unknown endpoint"})
            return

        try:
            length = int(self.headers.get("Content-Length", 0))

            if length <= 0 or length > 1_000_000:
                self.send_json(400, {"error": "Invalid request size"})
                return

            request = json.loads(
                self.rfile.read(length).decode("utf-8")
            )

            config = SafetyConfig(
                collision_distance=request.get(
                    "collision_distance", 0.25
                ),
                downwash_horizontal_radius=request.get(
                    "downwash_horizontal_radius", 0.40
                ),
                downwash_vertical_threshold=request.get(
                    "downwash_vertical_threshold", 0.60
                ),
            )

            result = plan(
                starts=request["starts"],
                goals=request["goals"],
                bounds=request["bounds"],
                resolution=request.get("resolution", 0.5),
                config=config,
                max_time=request.get("max_time", 80),
                max_nodes=request.get("max_nodes", 3000),
            )

            if result is None:
                self.send_json(422, {
                    "status": "no_solution",
                    "message": "No conflict-free path found",
                })
                return

            self.send_json(200, {
                "status": "success",
                **result,
            })

        except (KeyError, TypeError, ValueError) as exc:
            self.send_json(400, {"error": str(exc)})

        except Exception as exc:
            self.send_json(500, {"error": str(exc)})


if __name__ == "__main__":
    server = ThreadingHTTPServer(
        ("127.0.0.1", 5050),
        PlannerHandler,
    )

    print("3D CBS planner listening on http://127.0.0.1:5050",
          flush=True)

    server.serve_forever()
