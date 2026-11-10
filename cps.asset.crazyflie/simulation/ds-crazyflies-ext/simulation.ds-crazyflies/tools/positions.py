#!/usr/bin/env python3
"""Aggregate native odometry into the Path format consumed by FEDT Java."""
import json
from pathlib import Path
import time

from config import RUNTIME, load


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    config = load('/ds/crazywebotsworld/worlds/crazyflie.wbt')
    rclpy.init()
    node = Node('fedt_positions')
    poses = {}
    retained = []
    publisher = node.create_publisher(RosPath, '/cf_positions_path', 10)
    ready = RUNTIME / 'positions-ready.json'
    ready.unlink(missing_ok=True)
    for drone in config['drone_ids']:
        def receive(message, drone=drone):
            pose = PoseStamped()
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = f'cf{drone}'
            pose.pose = message.pose.pose
            poses[drone] = (time.monotonic(), pose)
        retained.append(node.create_subscription(Odometry, f'/cf{drone}/odom', receive, 10))
    def publish():
        now = time.monotonic()
        current = [poses[d][1] for d in config['drone_ids'] if d in poses and now-poses[d][0]<2]
        if not current:
            ready.unlink(missing_ok=True)
            return
        message = RosPath()
        message.header.frame_id = 'world'
        message.header.stamp = current[-1].header.stamp
        message.poses = current
        publisher.publish(message)
        if len(current)==len(config['drone_ids']):
            if not ready.exists():
                temporary = ready.with_suffix('.tmp')
                temporary.write_text(json.dumps(config['drone_ids']))
                temporary.replace(ready)
        else:
            ready.unlink(missing_ok=True)
    timer = node.create_timer(.05, publish)
    try:
        rclpy.spin(node)
    finally:
        ready.unlink(missing_ok=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
