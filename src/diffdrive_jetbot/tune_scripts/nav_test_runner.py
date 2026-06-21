#!/usr/bin/env python3
"""
Repeatable Nav2 test patterns through the SAME goal path the experiment uses:
Pose -> /<ns>/nav/goal -> robot_nav_bridge -> NavigateToPose, completion via
/<ns>/nav/goal_reached (Bool).

Modes:
  * step (default, experiment profile): a series of `--steps` sequential goals,
    `--step` metres apart along the starting heading — exactly how the robot hops
    cell-to-cell in the real run.  Then a 180-degree rotate-in-place goal and the
    same steps back, then rotate to the original heading.  `--laps` repetitions.
  * continuous: one single goal `--distance` metres ahead (straight-line weave
    diagnostics without per-cell stop-and-go), then 180 turn + return.

Goal positions are computed once from the START pose (start + k*step along the
initial heading), not chained from the current pose — target drift does not
accumulate, matching how the experiment derives goals from the grid.

Run INSIDE the robot container:
  python3 nav_test_runner.py --namespace robot_2 --step 0.3 --steps 3 --laps 3
  python3 nav_test_runner.py --namespace robot_2 --mode continuous --distance 0.9
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy
import tf2_ros
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class NavTestRunner(Node):
    """Sends goals to robot_nav_bridge and waits for completion."""

    def __init__(self, namespace: str):
        super().__init__('nav_test_runner')
        self._ns = namespace
        self._reached = None  # None = pending, True/False = bridge result

        # Namespaced TF: nodes publish to /<ns>/tf, the default TransformListener
        # only watches /tf — subscribe manually (same pattern as the main-comp
        # TFListener).
        self._tf_buffer = tf2_ros.Buffer()
        tf_static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            TFMessage, f'/{namespace}/tf',
            lambda m: self._tf_cb(m, False), 100)
        self.create_subscription(
            TFMessage, f'/{namespace}/tf_static',
            lambda m: self._tf_cb(m, True), tf_static_qos)

        self._goal_pub = self.create_publisher(Pose, f'/{namespace}/nav/goal', 10)
        self.create_subscription(
            Bool, f'/{namespace}/nav/goal_reached', self._reached_cb, 10)

    def _tf_cb(self, msg: TFMessage, is_static: bool) -> None:
        for tr in msg.transforms:
            if is_static:
                self._tf_buffer.set_transform_static(tr, 'runner')
            else:
                self._tf_buffer.set_transform(tr, 'runner')

    def _reached_cb(self, msg: Bool) -> None:
        self._reached = bool(msg.data)

    def current_pose(self, timeout_sec: float = 15.0):
        """Return (x, y, yaw) of base_footprint in the map frame."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tr = self._tf_buffer.lookup_transform(
                    f'{self._ns}/map', f'{self._ns}/base_footprint',
                    rclpy.time.Time(), timeout=Duration(seconds=0.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            t = tr.transform.translation
            q = tr.transform.rotation
            return t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w)
        raise RuntimeError('No map->base_footprint TF after %.0f s' % timeout_sec)

    def go(self, x: float, y: float, yaw: float, timeout_sec: float, label: str) -> float:
        """Send one goal and block until goal_reached. Returns elapsed seconds."""
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.orientation.z = math.sin(yaw / 2.0)
        goal.orientation.w = math.cos(yaw / 2.0)

        self._reached = None
        self._goal_pub.publish(goal)
        self.get_logger().info(f'[{label}] goal -> ({x:.3f}, {y:.3f}, {math.degrees(yaw):.0f} deg)')

        t0 = time.monotonic()
        while self._reached is None:
            if time.monotonic() - t0 > timeout_sec:
                raise RuntimeError(f'[{label}] goal timeout after {timeout_sec:.0f} s')
            rclpy.spin_once(self, timeout_sec=0.1)
        elapsed = time.monotonic() - t0
        if not self._reached:
            raise RuntimeError(f'[{label}] bridge reported goal FAILED')
        self.get_logger().info(f'[{label}] reached in {elapsed:.1f} s')
        return elapsed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='robot_2')
    parser.add_argument('--mode', choices=['step', 'continuous'], default='step')
    parser.add_argument('--step', type=float, default=0.3, help='cell size, m')
    parser.add_argument('--steps', type=int, default=3, help='cells per direction')
    parser.add_argument('--distance', type=float, default=0.9,
                        help='continuous mode: single-goal distance, m')
    parser.add_argument('--laps', type=int, default=3)
    parser.add_argument('--pause', type=float, default=2.0,
                        help='pause between goals, s')
    parser.add_argument('--goal-timeout', type=float, default=40.0)
    args = parser.parse_args()

    rclpy.init()
    node = NavTestRunner(args.namespace)
    times = []
    try:
        x0, y0, yaw0 = node.current_pose()
        node.get_logger().info(
            f'Start pose: ({x0:.3f}, {y0:.3f}, {math.degrees(yaw0):.0f} deg)')
        ca, sa = math.cos(yaw0), math.sin(yaw0)
        yaw_back = yaw0 + math.pi

        if args.mode == 'step':
            far = args.steps * args.step
            for lap in range(args.laps):
                # forward, cell by cell
                for k in range(1, args.steps + 1):
                    d = k * args.step
                    times.append(node.go(x0 + d * ca, y0 + d * sa, yaw0,
                                         args.goal_timeout, f'lap{lap+1}-fwd{k}'))
                    time.sleep(args.pause)
                # rotate in place at the far end
                node.go(x0 + far * ca, y0 + far * sa, yaw_back,
                        args.goal_timeout, f'lap{lap+1}-turnA')
                time.sleep(args.pause)
                # back, cell by cell
                for k in range(args.steps - 1, -1, -1):
                    d = k * args.step
                    times.append(node.go(x0 + d * ca, y0 + d * sa, yaw_back,
                                         args.goal_timeout, f'lap{lap+1}-back{args.steps-k}'))
                    time.sleep(args.pause)
                # rotate back to the original heading for the next lap
                node.go(x0, y0, yaw0, args.goal_timeout, f'lap{lap+1}-turnB')
                time.sleep(args.pause)
        else:
            for lap in range(args.laps):
                d = args.distance
                times.append(node.go(x0 + d * ca, y0 + d * sa, yaw0,
                                     args.goal_timeout, f'lap{lap+1}-fwd'))
                time.sleep(args.pause)
                node.go(x0 + d * ca, y0 + d * sa, yaw_back,
                        args.goal_timeout, f'lap{lap+1}-turnA')
                time.sleep(args.pause)
                times.append(node.go(x0, y0, yaw_back,
                                     args.goal_timeout, f'lap{lap+1}-back'))
                time.sleep(args.pause)
                node.go(x0, y0, yaw0, args.goal_timeout, f'lap{lap+1}-turnB')
                time.sleep(args.pause)

        if times:
            node.get_logger().info(
                f'Done: {len(times)} translation goals, '
                f'avg {sum(times)/len(times):.1f} s, max {max(times):.1f} s')
    except (KeyboardInterrupt, RuntimeError) as e:
        if not isinstance(e, KeyboardInterrupt):
            node.get_logger().error(str(e))
            sys.exit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
