#!/usr/bin/env python3
"""Native broadcast service facade, with one shared simulation start epoch."""
import argparse
import json
import threading


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone-ids", default=",".join(f"cf{i}" for i in range(231, 243)))
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
    from crazyflie_interfaces.srv import Takeoff, Land, GoTo, StartTrajectory, NotifySetpointsStop
    from std_srvs.srv import Trigger
    from std_msgs.msg import Float64MultiArray, String
    rclpy.init()
    node = Node("ds3_swarm_services", parameter_overrides=[Parameter("use_sim_time", value=True)])
    callback = ReentrantCallbackGroup()
    schedule = node.create_publisher(Float64MultiArray, "/_ds/trajectory_schedule", 10)
    report = node.create_publisher(String, "/all/control_result", 10)
    clients, retained = {}, []
    def call(drone, name, cls, request):
        key = drone, name
        if key not in clients:
            clients[key] = node.create_client(cls, f"/{drone}/{name}", callback_group=callback)
        client = clients[key]
        if not client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError(f"{drone}/{name} unavailable")
        future = client.call_async(request)
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(4):
            future.cancel()
            raise TimeoutError(f"{drone}/{name} timeout")
        return future.result()

    def execute(name, cls, request, response):
        try:
            if name == "start_trajectory":
                if request.timescale <= 0:
                    raise ValueError("timescale must be positive")
                for drone in args.drone_ids.split(","):
                    status = json.loads(call(drone, "trajectory_status", Trigger, Trigger.Request()).message)
                    if request.group_mask and not (status["group"] & request.group_mask):
                        continue
                    if request.trajectory_id not in status["ids"]:
                        raise ValueError(f"{drone}: trajectory {request.trajectory_id} not uploaded")
                epoch = node.get_clock().now().nanoseconds / 1e9 + 1.0
                message = Float64MultiArray()
                message.data = [float(request.trajectory_id), float(request.timescale),
                    float(request.reversed), float(request.relative), float(request.group_mask), epoch]
                schedule.publish(message)
                result = dict(action=name, success=True, start_time=epoch)
            else:
                for drone in args.drone_ids.split(","):
                    call(drone, name, cls, request)
                result = dict(action=name, success=True)
        except Exception as exc:
            result = dict(action=name, success=False, error=str(exc))
            node.get_logger().error(str(exc))
        message = String()
        message.data = json.dumps(result)
        report.publish(message)
        return response
    for name, cls in dict(takeoff=Takeoff, land=Land, go_to=GoTo,
                          start_trajectory=StartTrajectory, notify_setpoints_stop=NotifySetpointsStop).items():
        retained.append(node.create_service(cls, f"/all/{name}",
            lambda q, r, name=name, cls=cls: execute(name, cls, q, r), callback_group=callback))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
