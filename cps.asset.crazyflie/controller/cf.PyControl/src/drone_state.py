"""Passive per-drone state and asynchronous, atomic PNG publication."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import RLock
import logging
import os


PHASES = (
    "installed", "resolved", "starting", "active", "stopping", "uninstalled", "idle", "taking_off", "hovering", "navigating", "trajectory",
    "full_state", "mapping", "braking", "landing", "landed", "fault", "offline", "shutdown",
)
PUBLIC_STATE_NAMES = {"taking_off": "flying", "navigating": "flying", "trajectory": "flying",
                      "full_state": "flying", "mapping": "flying", "braking": "flying", "landed": "landing",
                      "fault": "error", "offline": "offline"}


class DroneState:
    def __init__(self, drone_id, image_dir=None):
        self.drone_id = drone_id
        self.phase = "active"
        self.transition = "initialize"
        self.error = None
        self._lock = RLock()
        self._revision = 0
        self._image_dir = Path(image_dir) / drone_id if image_dir else None
        self._renderer = ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"graph-{drone_id}")
        self._pending = None
        if self._image_dir:
            self._revision = 1
            self._pending = self._renderer.submit(self._render, self.phase, self._revision)

    def set(self, phase, transition, error=None):
        if phase not in PHASES:
            raise ValueError(f"Unknown phase: {phase}")
        with self._lock:
            changed = phase != self.phase
            self.phase, self.transition, self.error = phase, transition, error
            if changed and self._image_dir:
                self._revision += 1
                revision = self._revision
                # Coalesce superseded renders instead of blocking the ROS executor.
                if self._pending:
                    self._pending.cancel()
                self._pending = self._renderer.submit(self._render, phase, revision)

    def snapshot(self):
        with self._lock:
            return {"state": PUBLIC_STATE_NAMES.get(self.phase, self.phase), "phase": self.phase,
                    "transition": self.transition, "error": self.error}

    def _render(self, phase, revision):
        try:
            import pydot
            graph = pydot.Dot(graph_type="digraph", rankdir="LR", label=self.drone_id)
            for name in PHASES:
                graph.add_node(pydot.Node(name, style="filled", fillcolor="#8bd3b0" if name == phase else "white"))
            for a, b in [("active", "idle"), ("idle", "taking_off"), ("taking_off", "hovering"),
                         ("hovering", "landing"), ("landing", "landed"), ("landed", "idle")]:
                graph.add_edge(pydot.Edge(a, b))
            for name in ["navigating", "trajectory", "full_state", "mapping"]:
                graph.add_edge(pydot.Edge("hovering", name))
                graph.add_edge(pydot.Edge(name, "hovering"))
                graph.add_edge(pydot.Edge(name, "landing"))
                graph.add_edge(pydot.Edge(name, "braking"))
            graph.add_edge(pydot.Edge("braking", "hovering"))
            data = graph.create_png()
            self._image_dir.mkdir(parents=True, exist_ok=True)
            image = self._image_dir / f"uav{revision}.png"
            temporary = image.with_suffix(".tmp")
            temporary.write_bytes(data)
            os.replace(temporary, image)
            latest = self._image_dir / "latest.txt"
            tmp = self._image_dir / "latest.tmp"
            tmp.write_text(str(revision))
            os.replace(tmp, latest)
            # Bounded history; old webviews only need the most recent PNG.
            for old in self._image_dir.glob("uav*.png"):
                if old.stem[3:].isdigit() and not max(1, revision - 10) <= int(old.stem[3:]) <= revision:
                    old.unlink(missing_ok=True)
        except Exception:
            logging.exception("State graph failed for %s", self.drone_id)

    def close(self):
        self._renderer.shutdown(wait=True, cancel_futures=True)
