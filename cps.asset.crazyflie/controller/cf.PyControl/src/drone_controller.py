"""One command owner per drone. HTTP calls these methods directly; ROS is lazy.

No cflib/ROS imports are needed to test task ownership, queues or HTTP contracts.
RosTransport below is the only place that constructs ROS messages and clients.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
import threading
import time
import uuid

from drone_state import DroneState


class ControlError(RuntimeError):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


def finite(value, name="value"):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def vector(values, length=3):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f"Expected {length} numbers")
    return [finite(x) for x in values]


def positive(value, name="duration"):
    value = finite(value, name)
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


def integer(value, name, minimum, maximum):
    number = finite(value, name)
    if not number.is_integer() or not minimum <= number <= maximum:
        raise ValueError(f"{name} must be an integer in {minimum}..{maximum}")
    return int(number)


def validate_pieces(pieces):
    if not isinstance(pieces, list) or not 1 <= len(pieces) <= 4096:
        raise ValueError("pieces must contain 1..4096 polynomial segments")
    result = []
    for p in pieces:
        piece = {"duration": positive(p["duration"])}
        for key in ("poly_x", "poly_y", "poly_z", "poly_yaw"):
            coefficients = p[key]
            if not isinstance(coefficients, list) or not 1 <= len(coefficients) <= 8:
                raise ValueError(f"{key} must contain 1..8 coefficients, constant first")
            piece[key] = [finite(x) for x in coefficients] + [0.0] * (8 - len(coefficients))
        result.append(piece)
    return result


def evaluate(pieces, elapsed, timescale=1.0, reverse=False):
    """Polynomial reference, including derivative scaling and reverse signs."""
    duration = sum(p["duration"] for p in pieces)
    tau = min(max(elapsed / timescale, 0.0), duration)
    if reverse:
        tau = duration - tau
    p = pieces[-1]
    for index, piece in enumerate(pieces):
        p = piece
        # Keep the final piece selected when duration sums round slightly high.
        if tau <= p["duration"] or index == len(pieces) - 1:
            break
        tau -= p["duration"]
    tau = min(tau, p["duration"])
    sign = -1.0 if reverse else 1.0
    def derivative(coeffs, order):
        return sum(coeffs[i] * math.prod(range(i - order + 1, i + 1)) * tau ** (i - order)
                   for i in range(order, len(coeffs))) * (sign / timescale) ** order
    axes = [p[k] for k in ("poly_x", "poly_y", "poly_z")]
    return {"position": [derivative(c, 0) for c in axes],
            "velocity": [derivative(c, 1) for c in axes],
            "acceleration": [derivative(c, 2) for c in axes],
            "yaw": derivative(p["poly_yaw"], 0),
            "omega": [0.0, 0.0, derivative(p["poly_yaw"], 1)]}


class DroneController:
    def __init__(self, drone_id, transport, image_dir=None, goal_tolerance=0.08,
                 telemetry_timeout=2.0, operation_timeout=60.0, brake_acceleration=0.5):
        self.drone_id, self.transport = drone_id, transport
        self.state = DroneState(drone_id, image_dir)
        self.goal_tolerance = goal_tolerance
        self.telemetry_timeout = telemetry_timeout
        self.operation_timeout = operation_timeout
        self.brake_acceleration = positive(brake_acceleration, "brake_acceleration")
        self._lock = threading.RLock()
        self._commands = threading.RLock()
        self._uploads = threading.RLock()
        self._pending_trajectory = None
        self._vehicle_trajectories = set()
        self._active_trajectory = None
        self._holding = None
        self._streaming = False
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix=drone_id)
        self._changed = threading.Event()
        self._cancel = threading.Event()
        self._tasks, self._trajectories = {}, {}
        self._current = None
        self._generation = 0
        self._queue = deque()
        self.active_target = None
        self.telemetry = None
        self._last_telemetry = None
        self._last_wall_telemetry = None
        self._initial_position = None
        self._hold_target = None
        self._external_action = None
        self._wall = None
        self._closed = False
        self._shutdown = threading.Event()
        self._disconnected_phase = "active"
        transport.on_telemetry = self.update_telemetry
        self._watchdog = threading.Thread(target=self._watch_connection, daemon=True, name=f"link-{drone_id}")
        self._watchdog.start()
        self._holder = threading.Thread(target=self._hold_loop, daemon=True, name=f"hold-{drone_id}")
        self._holder.start()

    def _yaw(self):
        x, y, z, w = self.telemetry["orientation"]
        return math.atan2(2 * (w*z+x*y), 1-2*(y*y+z*z))

    def _hold_loop(self):
        while not self._shutdown.wait(0.02):
            with self._commands:
                if self._holding is None:
                    continue
                try:
                    target, yaw = self._holding
                    self.transport.command("hold", goal=target, yaw=yaw)
                except Exception as exc:
                    self._holding = None
                    self._cancel.set()
                    self.state.set("fault", "hold_failed", str(exc))

    def _set_hold(self, cancel, target, yaw):
        with self._commands:
            if cancel.is_set():
                raise InterruptedError()
            self.transport.command("hold", goal=target, yaw=yaw)
            self._holding = (target[:], yaw)
            self._hold_target = target[:]
            self._streaming = True

    def _publish_reference(self, cancel, sample):
        with self._commands:
            if cancel.is_set():
                raise InterruptedError()
            self._holding = None
            self.transport.publish_full_state(sample)
            self._streaming = True

    def _watch_connection(self):
        while not self._shutdown.wait(0.2):
            with self._lock:
                if self.telemetry is None or self.state.phase in ("fault", "shutdown"):
                    continue
                if not self._fresh() and self.state.phase != "offline":
                    self._disconnected_phase = self.state.phase
                    self.state.set("offline", "telemetry_lost")
                elif self._fresh() and self.state.phase == "offline":
                    phase = self._disconnected_phase
                    self.state.set(phase if phase in ("idle", "active", "hovering", "landed") else "hovering", "telemetry_restored")

    def update_telemetry(self, position, velocity=None, orientation=None, **extra):
        with self._lock:
            position = vector(position)
            self.telemetry = dict(position=position, velocity=vector(velocity or [0, 0, 0]),
                                  orientation=vector(orientation or [0, 0, 0, 1], 4), **extra)
            self._last_telemetry = self.transport.now()
            self._last_wall_telemetry = time.monotonic()
            if self._initial_position is None:
                self._initial_position = position[:]
            if self._external_action and not (self._current and self._tasks[self._current]["status"] in ("queued", "running")):
                mode = extra.get("motion", {}).get("mode")
                if mode in (0, 1):
                    phase = "hovering" if position[2] > 0.10 else "landed"
                else:
                    phase = {2: {"takeoff": "taking_off", "land": "landing"}.get(self._external_action, "navigating"),
                             3: "full_state", 4: "hovering", 5: "trajectory", 6: "navigating"}.get(mode)
                if phase:
                    self.state.set(phase, "external_ros")
        self._changed.set()

    def snapshot(self):
        with self._lock:
            return dict(self.state.snapshot(), drone_id=self.drone_id,
                        sim_time=self.transport.now(),
                        clock_type="simulation" if getattr(self.transport, "use_sim_time", False) else "wall",
                        backend=getattr(self.transport, "backend", "test"),
                        services_ready=getattr(self.transport, "services_ready", lambda: True)(),
                        queue_length=len(self._queue), active_target=copy.deepcopy(self.active_target),
                        task_id=self._current, telemetry=copy.deepcopy(self.telemetry),
                        connected=self._fresh())

    def task(self, task_id):
        with self._lock:
            if task_id not in self._tasks:
                raise ControlError("Unknown task", 404)
            return copy.deepcopy(self._tasks[task_id])

    def _fresh(self):
        return (self._last_telemetry is not None
                and self.transport.now() - self._last_telemetry <= self.telemetry_timeout
                and (getattr(self.transport, "use_sim_time", False)
                     or time.monotonic() - self._last_wall_telemetry <= self.telemetry_timeout))

    def _position(self):
        with self._lock:
            if not self._fresh():
                raise ControlError("Fresh position telemetry required", 503)
            return self.telemetry["position"][:]

    def _send(self, cancel, name, **payload):
        # Serialize dispatch with cancellation; a previous command cannot be sent
        # after its replacement's stop/hold command.
        with self._commands:
            if cancel.is_set():
                raise InterruptedError()
            if name in ("takeoff", "land", "go_to", "start_trajectory"):
                self._holding = None
                if self._streaming:
                    self.transport.command("notify_setpoints_stop", remain_valid_millisecs=0)
                    self._streaming = False
            self.transport.command(name, **payload)

    def _submit(self, action, phase, function):
        with self._lock:
            if self._closed:
                raise ControlError("Controller is closed", 503)
            if self._current and self._tasks[self._current]["status"] in ("queued", "running"):
                raise ControlError("Drone already has a motion task")
            task_id = uuid.uuid4().hex
            cancel = self._cancel = threading.Event()
            self._current = task_id
            self._external_action = None
            self._generation += 1
            generation = self._generation
            self._tasks[task_id] = dict(task_id=task_id, action=action, status="queued", error=None,
                                       created_at=self.transport.now())
            self.state.set(phase, action)
            self._worker.submit(self._run, task_id, generation, cancel, function)
            return task_id

    def _run(self, task_id, generation, cancel, function):
        try:
            with self._lock:
                if cancel.is_set():
                    return
                self._tasks[task_id]["status"] = "running"
                self._tasks[task_id]["started_at"] = self.transport.now()
            while True:
                finish_phase = function(cancel)
                with self._lock:
                    if not cancel.is_set() and generation == self._generation:
                        # An append may arrive just as the navigation worker
                        # observes an empty queue; drain it before completing.
                        if self._tasks[task_id]["action"] == "navigate" and self._queue:
                            continue
                        self._tasks[task_id]["status"] = "completed"
                        if self.active_target is not None:
                            self._hold_target = copy.deepcopy(self.active_target)
                        self._active_trajectory = None
                        self.active_target = None
                        self.state.set(finish_phase or "hovering", "completed")
                    return
        except InterruptedError:
            pass
        except Exception as exc:
            try:
                with self._commands:
                    if not cancel.is_set():
                        if self._fresh():
                            self._set_hold(cancel, self._position(), self._yaw())
            except Exception:
                pass
            with self._lock:
                if generation == self._generation and not cancel.is_set():
                    self._tasks[task_id].update(status="failed", error=str(exc))
                    self.active_target = None
                    self._active_trajectory = None
                    self._queue.clear()
                    self.state.set("fault", "failed", str(exc))

    def _wait_goal(self, cancel, target, timeout=None):
        start = self.transport.now()
        settled = None
        while not cancel.is_set():
            with self._lock:
                current = self._position()
                if (math.dist(current, target) <= self.goal_tolerance
                        and math.sqrt(sum(v*v for v in self.telemetry["velocity"])) <= 0.15):
                    if settled is None:
                        settled = self.transport.now()
                    elif self.transport.now() - settled >= 0.2:
                        return
                else:
                    settled = None
            if self.transport.now() - start > (timeout or self.operation_timeout):
                raise TimeoutError("Target position was not reached")
            if self.transport.now() < start - 0.001:
                raise ControlError("Simulation clock reset during motion", 503)
            self._changed.wait(0.02)
            self._changed.clear()
        raise InterruptedError()

    def activate_idle(self):
        with self._lock:
            if self._current and self._tasks[self._current]["status"] in ("queued", "running"):
                raise ControlError("Cannot activate idle during a task")
            self.state.set("idle", "activate_idle")
            self._external_action = None

    def lifecycle(self, event):
        phases = dict(install="resolved", start="starting", initialize="active", stop="stopping",
                      uninstall="uninstalled", shutdown_command="shutdown", begin_stopping="stopping")
        if event in ("stop", "uninstall", "shutdown_command", "begin_stopping"):
            self.stop()
        self.state.set(phases[event], event)

    def takeoff(self, height=0.5, duration=2.0):
        height, duration = positive(height, "height"), positive(duration)
        with self._lock:
            if self.state.phase not in ("idle", "landed"):
                raise ControlError("Takeoff requires idle or landed state")
            target = self._position()
            target[2] = height
        def fly(cancel):
            self.active_target = target
            self._send(cancel, "takeoff", height=height, duration=duration)
            self._wait_goal(cancel, target)
        return self._submit("begin_takeoff", "taking_off", fly)

    def navigate(self, target, append=False, duration=10.0):
        target, duration = vector(target), positive(duration)
        with self._lock:
            if self.state.phase not in ("hovering", "navigating"):
                raise ControlError("Navigation requires hovering or navigating state")
            self._queue.append((target, duration))
            if append or self.state.phase == "navigating":
                return self._current
            def fly(cancel):
                while not cancel.is_set():
                    with self._lock:
                        if not self._queue:
                            return "hovering"
                        point, seconds = self._queue.popleft()
                        self.active_target = point
                    self._send(cancel, "go_to", goal=point, duration=seconds)
                    self._wait_goal(cancel, point, max(self.operation_timeout, seconds * 2))
                raise InterruptedError()
            return self._submit("navigate", "navigating", fly)

    def stop(self, release=True):
        # Cancellation and the last old write share the same dispatch lock.
        with self._commands:
            with self._lock:
                self._cancel.set()
                self._generation += 1
                if self._current and self._tasks[self._current]["status"] in ("queued", "running"):
                    self._tasks[self._current]["status"] = "cancelled"
                self._queue.clear()
                self.active_target = None
                self._active_trajectory = None
            self._holding = None
            if release:
                self.transport.command("notify_setpoints_stop", remain_valid_millisecs=0)
                self._streaming = False
        self._changed.set()
        if self._wall:
            self._wall.stop()
            self._wall = None

    def hover(self, target=None, yaw=None):
        position = self._position()
        velocity = self.telemetry["velocity"][:]
        yaw = self._yaw() if yaw is None else finite(yaw, "yaw")
        target = vector(target) if target is not None else None
        speed = math.sqrt(sum(v*v for v in velocity))
        # Cubic velocity decay gives continuous velocity/acceleration. This is
        # a braking reference, not a guarantee of obstacle-free stopping space.
        duration = max(0.2, 1.5 * speed / self.brake_acceleration)
        braking = speed > 0.05
        end = [position[i] + velocity[i] * duration / 2 for i in range(3)] if braking else position
        if target is not None and math.dist(target, end) > self.goal_tolerance:
            raise ControlError("Hover target must be near the braking endpoint; use move/realtime to relocate")
        end = target or end
        self.stop(release=False)
        def hold(cancel):
            self.active_target = end
            if braking:
                start = self.transport.now()
                while self.transport.now() - start < duration:
                    if cancel.is_set():
                        raise InterruptedError()
                    self._position()
                    u = max(0.0, min(1.0, (self.transport.now()-start)/duration))
                    self._publish_reference(cancel, dict(
                        position=[position[i]+velocity[i]*duration*(u-u**3+0.5*u**4) for i in range(3)],
                        velocity=[v*(1-3*u*u+2*u**3) for v in velocity],
                        acceleration=[v*(-6*u+6*u*u)/duration for v in velocity],
                        yaw=yaw, omega=[0, 0, 0]))
                    cancel.wait(.02)
            self._set_hold(cancel, end, yaw)
            self._wait_goal(cancel, end)
        return self._submit("hover", "braking", hold)

    def land(self, height=0.0, duration=2.0):
        height, duration = finite(height, "height"), positive(duration)
        target = self._position()
        target[2] = height + 0.02
        self.stop(release=False)
        def descend(cancel):
            self.active_target = target
            self._send(cancel, "land", height=height, duration=duration)
            self._wait_goal(cancel, target)
            return "landed"
        return self._submit("begin_landing", "landing", descend)

    def move(self, target, velocity=0.1, timeout=60.0):
        target, velocity, timeout = vector(target), positive(velocity, "velocity"), positive(timeout, "timeout")
        self._position()
        self.stop(release=False)
        def fly(cancel):
            self.active_target = target
            yaw = self._yaw()
            last = start = self.transport.now()
            previous = self.telemetry["velocity"][:]
            with self._commands:
                self._holding = None
            while not cancel.is_set():
                current = self._position()
                now = self.transport.now()
                if now < last:
                    raise ControlError("Controller clock reset", 503)
                dt = min(.1, max(0.0, now-last))
                error = [target[i]-current[i] for i in range(3)]
                distance = math.sqrt(sum(x*x for x in error))
                limit = min(velocity, math.sqrt(2*self.brake_acceleration*distance), distance*1.5)
                desired = [x/distance*limit for x in error] if distance else [0, 0, 0]
                delta = [desired[i]-previous[i] for i in range(3)]
                norm = math.sqrt(sum(x*x for x in delta))
                scale = min(1.0, self.brake_acceleration*dt/norm) if norm else 1.0
                previous = [previous[i]+delta[i]*scale for i in range(3)]
                with self._commands:
                    if cancel.is_set():
                        raise InterruptedError()
                    self.transport.publish_velocity(previous, 0.0)
                    self._streaming = True
                if distance <= self.goal_tolerance and math.sqrt(sum(v*v for v in previous)) <= .05:
                    break
                if now-start > timeout:
                    raise TimeoutError("Real-time movement timed out")
                last = now
                cancel.wait(.02)
            self._set_hold(cancel, target, yaw)
            self._wait_goal(cancel, target, timeout)
        return self._submit("move/realtime", "navigating", fly)

    def upload(self, trajectory_id, pieces, piece_offset=0, upload_to_vehicle=True):
        if not isinstance(upload_to_vehicle, bool):
            raise ValueError("upload_to_vehicle must be a boolean")
        with self._uploads:
            if self._pending_trajectory == int(trajectory_id) or self._active_trajectory == int(trajectory_id):
                raise ControlError("Cannot replace an active trajectory; stop it first")
            return self._upload(trajectory_id, pieces, piece_offset, upload_to_vehicle)

    def _upload(self, trajectory_id, pieces, piece_offset=0, upload_to_vehicle=True):
        trajectory_id = integer(trajectory_id, "trajectory_id", 0, 255)
        piece_offset = integer(piece_offset, "piece_offset", 0, 4096)
        pieces = validate_pieces(pieces)
        previous = copy.deepcopy(self._trajectories.get(trajectory_id, []))
        if piece_offset > len(previous):
            raise ValueError("piece_offset would create a gap")
        updated = previous[:] if piece_offset else []
        updated[piece_offset:piece_offset + len(pieces)] = pieces
        if upload_to_vehicle:
            # Never hold the flight dispatch lock during a radio memory write.
            self.transport.command("upload_trajectory", trajectory_id=trajectory_id,
                                   piece_offset=0, pieces=updated)
            self._vehicle_trajectories.add(trajectory_id)
        else:
            self._vehicle_trajectories.discard(trajectory_id)
        self._trajectories[trajectory_id] = updated

    def start_trajectory(self, trajectory_id=0, timescale=1.0, reverse=False,
                         relative=True, full_state=False, rate=30.0):
        with self._uploads:
            self._pending_trajectory = int(trajectory_id)
            try:
                return self._start_trajectory(trajectory_id, timescale, reverse, relative, full_state, rate)
            except Exception:
                self._pending_trajectory = None
                raise

    def _start_trajectory(self, trajectory_id, timescale, reverse, relative, full_state, rate):
        timescale, rate = positive(timescale, "timescale"), positive(rate, "rate")
        if rate > 200:
            raise ValueError("rate must be <= 200 Hz")
        trajectory_id = integer(trajectory_id, "trajectory_id", 0, 255)
        if not isinstance(reverse, bool) or not isinstance(relative, bool):
            raise ValueError("reversed and relative must be booleans")
        if trajectory_id not in self._trajectories:
            raise ValueError("Trajectory has not been uploaded through this controller")
        if not full_state and trajectory_id not in self._vehicle_trajectories:
            raise ControlError("Trajectory is cached for FullState only; upload to vehicle before native start")
        if self.state.phase != "hovering":
            raise ControlError("Trajectory requires hovering state")
        pieces = copy.deepcopy(self._trajectories[trajectory_id])
        measured = self._position()
        position = self._hold_target or measured
        initial = evaluate(pieces, 0, timescale, reverse)["position"]
        initial_velocity = evaluate(pieces, 0, timescale, reverse)["velocity"]
        if math.sqrt(sum(v*v for v in initial_velocity)) > .15:
            raise ControlError("Trajectory must start near rest; provide a smooth connection from hover")
        if not relative and math.dist(measured, initial) > self.goal_tolerance:
            raise ControlError("Absolute trajectory must start at the current hovering position")
        offset = [position[i] - initial[i] for i in range(3)] if relative else [0, 0, 0]
        total = sum(p["duration"] for p in pieces) * timescale
        final = evaluate(pieces, total, timescale, reverse)["position"]
        target = [final[i] + offset[i] for i in range(3)]
        def fly(cancel):
            self.active_target = target
            self._active_trajectory = trajectory_id
            try:
                if not full_state:
                    self._send(cancel, "start_trajectory", trajectory_id=trajectory_id,
                               timescale=timescale, reversed=bool(reverse), relative=bool(relative))
            finally:
                with self._uploads:
                    if self._pending_trajectory == trajectory_id:
                        self._pending_trajectory = None
            start = self.transport.now()
            with self._lock:
                if cancel.is_set():
                    raise InterruptedError()
                self._tasks[self._current]["trajectory_started_at"] = start
            next_send = start
            while not cancel.is_set() and self.transport.now() - start < total:
                self._position()
                now = self.transport.now()
                if full_state and now >= next_send:
                    sample = evaluate(pieces, now - start, timescale, reverse)
                    sample["position"] = [sample["position"][i] + offset[i] for i in range(3)]
                    self._publish_reference(cancel, sample)
                    next_send = now + 1.0 / rate
                cancel.wait(0.005)
            if cancel.is_set():
                raise InterruptedError()
            if full_state:
                self._set_hold(cancel, target, evaluate(pieces, total, timescale, reverse)["yaw"])
            self._wait_goal(cancel, target)
        return self._submit("full_state" if full_state else "trajectory",
                            "full_state" if full_state else "trajectory", fly)

    def start_wall_follow(self, data):
        if self.state.phase != "hovering":
            raise ControlError("Wall following requires hovering state")
        from wall_following import WallFollowingConfig, WallFollowingOrchestrator
        data = dict(data)
        if self.state._image_dir:
            data.setdefault("image_output_base_dir", str(self.state._image_dir.parent.parent / "wall_following/img"))
        config = WallFollowingConfig.from_request(robot_prefix=f"/{self.drone_id}", request_data=data)
        config.use_sim_time = getattr(self.transport, "use_sim_time", False)
        self.stop(release=False)
        self._wall = WallFollowingOrchestrator(self.transport.executor)
        if not self._wall.start(config):
            self._wall = None
            raise ControlError("Wall following node failed to start", 503)
        self._streaming = True
        def follow(cancel):
            while not cancel.wait(0.05):
                self._position()
            raise InterruptedError()
        return self._submit("wall_follow/start", "mapping", follow)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._shutdown.set()
        self._watchdog.join(timeout=1)
        self._holder.join(timeout=2)
        self._cancel.set()
        self._changed.set()
        if self._wall:
            self._wall.stop()
        self._worker.shutdown(wait=True, cancel_futures=True)
        self.state.close()
        self.transport.close()


class RosTransport:
    """Owns a dedicated ROS executor. Never imports another overlay or cflib."""
    def __init__(self, drone_id, use_sim_time=True, require_report=False, backend=None):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.executors import MultiThreadedExecutor
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from crazyflie_interfaces.msg import Status
        from std_msgs.msg import String
        self.rclpy, self.drone_id = rclpy, drone_id
        self.use_sim_time = use_sim_time
        self.backend = backend or ("webots" if use_sim_time else "hardware")
        self.node = Node(f"http_{drone_id}", parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
        self.on_telemetry = None
        self._clients, self._publishers = {}, {}
        self._rpc_lock = threading.RLock()
        self._upload_rpc_lock = threading.RLock()
        self._report_ready = threading.Condition()
        self._reports = {}
        self._report_count = 0
        self.require_report = require_report
        self._sensors = {}
        self.node.create_subscription(Odometry, f"/{drone_id}/odom", self._odom, 10)
        self.node.create_subscription(LaserScan, f"/{drone_id}/scan", self._scan, 10)
        self.node.create_subscription(Status, f"/{drone_id}/status", self._status, 10)
        self.node.create_subscription(String, f"/{drone_id}/control_result", self._control_result, 10)
        self.node.create_subscription(String, f"/{drone_id}/motion_status", self._motion_status, 10)
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def services_ready(self):
        services = dict(self.node.get_service_names_and_types())
        ready = all(f"/{self.drone_id}/{name}" in services for name in
                    ("takeoff", "land", "go_to", "notify_setpoints_stop", "upload_trajectory", "start_trajectory"))
        if self.backend == "hardware":
            ready = ready and self.node.count_publishers(f"/{self.drone_id}/control_result") > 0
            ready = ready and self.node.count_subscribers(f"/{self.drone_id}/cmd_velocity_world") > 0
        return ready

    def _odom(self, msg):
        if self.on_telemetry:
            p, v, q = msg.pose.pose.position, msg.twist.twist.linear, msg.pose.pose.orientation
            self._orientation = [q.x, q.y, q.z, q.w]
            velocity = [v.x, v.y, v.z]
            if self.backend == "hardware":
                # The cflib Kalman log exposes body-frame velocity; motion
                # planning uses world coordinates. Webots already sends world v.
                u = [q.x, q.y, q.z]
                def cross(a, b):
                    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
                uv = cross(u, velocity)
                uuv = cross(u, uv)
                velocity = [velocity[i]+2*q.w*uv[i]+2*uuv[i] for i in range(3)]
            self.on_telemetry(position=[p.x, p.y, p.z], velocity=velocity,
                              orientation=[q.x, q.y, q.z, q.w], **self._sensors)

    def _scan(self, msg):
        self._sensors["scan"] = dict(angle_min=msg.angle_min, angle_max=msg.angle_max,
            angle_increment=msg.angle_increment, range_min=msg.range_min, range_max=msg.range_max,
            ranges=[float(r) if math.isfinite(r) else None for r in msg.ranges])

    def _status(self, msg):
        self._sensors["battery"] = float(msg.battery_voltage)
        self._sensors["pm_state"] = int(msg.pm_state)

    def _control_result(self, msg):
        try:
            result = json.loads(msg.data)
            with self._report_ready:
                self._report_count += 1
                self._reports[result["action"]] = (self._report_count, result)
                self._report_ready.notify_all()
        except (ValueError, KeyError):
            self.node.get_logger().error("Invalid control_result diagnostic")

    def _motion_status(self, msg):
        try:
            self._sensors["motion"] = json.loads(msg.data)
        except ValueError:
            self.node.get_logger().error("Invalid motion_status diagnostic")

    def _publish(self, name, type_name, data):
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.set_message import set_message_fields
        cls = get_message(type_name)
        if name not in self._publishers:
            self._publishers[name] = self.node.create_publisher(cls, f"/{self.drone_id}/{name}", 10)
        msg = cls()
        set_message_fields(msg, data)
        if hasattr(msg, "header") and not (msg.header.stamp.sec or msg.header.stamp.nanosec):
            msg.header.stamp = self.node.get_clock().now().to_msg()
        self._publishers[name].publish(msg)

    def command(self, name, **data):
        if name == "hold":
            return self._command(name, **data)
        with self._upload_rpc_lock if name == "upload_trajectory" else self._rpc_lock:
            return self._command(name, **data)

    def _command(self, name, **data):
        from rosidl_runtime_py.utilities import get_service
        from rosidl_runtime_py.set_message import set_message_fields
        if name == "hold":
            x, y, z = data["goal"]
            self._publish("cmd_position", "crazyflie_interfaces/msg/Position",
                          dict(x=x, y=y, z=z, yaw=math.degrees(data.get("yaw", 0))))
            return
        if "duration" in data:
            if not isinstance(data["duration"], dict):
                data["duration"] = self._duration(data["duration"])
        if "goal" in data:
            if not isinstance(data["goal"], dict):
                data["goal"] = dict(zip(("x", "y", "z"), data["goal"]))
        if "pieces" in data:
            data["pieces"] = [dict(p, duration=p["duration"] if isinstance(p["duration"], dict)
                                  else self._duration(p["duration"])) for p in data["pieces"]]
        types = dict(takeoff="Takeoff", land="Land", go_to="GoTo", upload_trajectory="UploadTrajectory",
                     start_trajectory="StartTrajectory", notify_setpoints_stop="NotifySetpointsStop",
                     add_crazyflie="AddCrazyflie", remove_crazyflie="RemoveCrazyflie",
                     set_group_mask="SetGroupMask", stop="Stop")
        if name not in self._clients:
            cls = get_service(f"crazyflie_interfaces/srv/{types[name]}")
            prefix = "/crazyflie_webots_gateway" if name in ("add_crazyflie", "remove_crazyflie") else f"/{self.drone_id}"
            self._clients[name] = self.node.create_client(cls, f"{prefix}/{name}")
        client = self._clients[name]
        if not client.wait_for_service(timeout_sec=1.0):
            raise ControlError(f"/{self.drone_id}/{name} unavailable", 503)
        req = client.srv_type.Request()
        # Legacy Takeoff/Land include use_current_yaw; new Services encode yaw.
        if data.pop("use_current_yaw", False) and self.on_telemetry:
            # Existing clients normally leave this false. Preserve semantics
            # using the latest measured orientation when explicitly requested.
            current = getattr(self, "_orientation", [0, 0, 0, 1])
            x, y, z, w = current
            data["yaw"] = math.atan2(2 * (w*z+x*y), 1-2*(y*y+z*z))
        data.pop("linear", None)
        set_message_fields(req, data)
        with self._report_ready:
            report_before = self._report_count
        future = client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(35 if name == "upload_trajectory" else 5):
            future.cancel()
            raise TimeoutError(f"ROS service {name} did not respond")
        response = future.result()
        if response is None:
            raise RuntimeError(f"ROS service {name} failed")
        if hasattr(response, "success") and not response.success:
            raise ControlError(getattr(response, "message", "ROS command rejected"), 400)
        if ((self.require_report and name in ("takeoff", "land", "go_to", "upload_trajectory", "start_trajectory"))
                or (self.backend == "hardware" and name == "upload_trajectory")):
            with self._report_ready:
                if not self._report_ready.wait_for(lambda: self._reports.get(name, (0, None))[0] > report_before, timeout=2):
                    raise TimeoutError(f"Missing execution acknowledgement for {name}")
                result = self._reports[name][1]
            if not result.get("success"):
                raise ControlError(result.get("error", "ROS command rejected"), 400)

    @staticmethod
    def _duration(seconds):
        n = round(positive(seconds) * 1_000_000_000)
        return dict(sec=n // 1_000_000_000, nanosec=n % 1_000_000_000)

    def publish_full_state(self, sample):
        yaw = sample["yaw"]
        self._publish("cmd_full_state", "crazyflie_interfaces/msg/FullState", {
            "pose": {"position": dict(zip(("x", "y", "z"), sample["position"])),
                     "orientation": dict(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))},
            "twist": {"linear": dict(zip(("x", "y", "z"), sample["velocity"])),
                      "angular": dict(zip(("x", "y", "z"), sample["omega"]))},
            "acc": dict(zip(("x", "y", "z"), sample["acceleration"]))})

    def publish_velocity(self, velocity, yaw_rate=0.0):
        # Both cflib and the Webots wire interface use degrees/s here.
        self._publish("cmd_velocity_world", "crazyflie_interfaces/msg/VelocityWorld",
                      dict(vel=dict(zip(("x", "y", "z"), velocity)), yaw_rate=math.degrees(yaw_rate)))

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self._thread.join(timeout=2)
        self.node.destroy_node()
