#!/usr/bin/env python3
"""
Motion telemetry logger for Nav2 tuning.

Records, per odometry message, the robot pose/velocity together with the latest
command from each stage of the velocity pipeline:

    DWB (cmd_vel_nav) -> velocity_smoother (cmd_vel) -> twist_mux
        -> diffbot_base_controller/cmd_vel_unstamped (final motor command)

plus segment markers from the nav goal flow (nav/goal -> nav/goal_reached), so the
analyzer can split the log into per-goal segments.

CSV columns:
  t, x, y, yaw, v_odom, w_odom, vx_nav, wz_nav, vx_smooth, wz_smooth,
  vx_final, wz_final, seg_id, goal_active

Run INSIDE the robot container:
  python3 motion_logger.py --namespace robot_2 --output logs/run_baseline.csv
Stop with Ctrl-C (or SIGTERM) — the CSV is flushed on every row.
"""

import argparse
import csv
import math
import os
import signal
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class MotionLogger(Node):
    def __init__(self, namespace: str, output_path: str):
        super().__init__('motion_logger')

        self._file = open(output_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            't', 'x', 'y', 'yaw', 'v_odom', 'w_odom',
            'vx_nav', 'wz_nav', 'vx_smooth', 'wz_smooth',
            'vx_final', 'wz_final', 'seg_id', 'goal_active',
        ])

        # Latest command values from each pipeline stage
        self._nav = (0.0, 0.0)
        self._smooth = (0.0, 0.0)
        self._final = (0.0, 0.0)
        self._seg_id = -1
        self._goal_active = 0
        self._rows = 0

        ns = namespace
        self.create_subscription(
            Odometry, f'/{ns}/diffbot_base_controller/odom', self._odom_cb, 50)
        self.create_subscription(
            Twist, f'/{ns}/cmd_vel_nav', self._mk_twist_cb('_nav'), 50)
        self.create_subscription(
            Twist, f'/{ns}/cmd_vel', self._mk_twist_cb('_smooth'), 50)
        self.create_subscription(
            Twist, f'/{ns}/diffbot_base_controller/cmd_vel_unstamped',
            self._mk_twist_cb('_final'), 50)
        self.create_subscription(Pose, f'/{ns}/nav/goal', self._goal_cb, 10)
        self.create_subscription(Bool, f'/{ns}/nav/goal_reached', self._reached_cb, 10)

        self.get_logger().info(f'Logging {ns} motion to {output_path}')

    def _mk_twist_cb(self, attr):
        def cb(msg: Twist):
            setattr(self, attr, (msg.linear.x, msg.angular.z))
        return cb

    def _goal_cb(self, msg: Pose) -> None:
        self._seg_id += 1
        self._goal_active = 1
        self.get_logger().info(
            f'segment {self._seg_id} start: goal '
            f'({msg.position.x:.3f}, {msg.position.y:.3f})')

    def _reached_cb(self, msg: Bool) -> None:
        self._goal_active = 0
        self.get_logger().info(
            f'segment {self._seg_id} end: goal_reached={msg.data}')

    def _odom_cb(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._writer.writerow([
            f'{t:.4f}',
            f'{p.x:.5f}', f'{p.y:.5f}',
            f'{yaw_from_quat(q.x, q.y, q.z, q.w):.5f}',
            f'{msg.twist.twist.linear.x:.5f}', f'{msg.twist.twist.angular.z:.5f}',
            f'{self._nav[0]:.5f}', f'{self._nav[1]:.5f}',
            f'{self._smooth[0]:.5f}', f'{self._smooth[1]:.5f}',
            f'{self._final[0]:.5f}', f'{self._final[1]:.5f}',
            self._seg_id, self._goal_active,
        ])
        self._file.flush()
        self._rows += 1

    def close(self) -> None:
        self._file.close()
        self.get_logger().info(f'Saved {self._rows} rows')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='robot_2')
    parser.add_argument('--output', default='motion_log.csv')
    args = parser.parse_args()

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    node = MotionLogger(args.namespace, args.output)

    # Graceful stop on SIGTERM as well as Ctrl-C
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
