#!/usr/bin/env python3
"""
Odometry calibration for the JetBot diff drive (wheel_radius / wheel_separation
multipliers in diffbot_controllers.yaml).

The robot drives a known amount BY ODOMETRY; you measure how far/how much it ACTUALLY
moved; the script computes the corrected multiplier.

Calibrate in this order (omega depends on wheel radius, so radius goes first):

  1. LINEAR (wheel_radius multiplier)
     Place the robot at a tape-measure zero mark, then:
       python3 calibrate_odometry.py drive --namespace robot_2 --distance 1.0
     The robot drives until odometry says 1.0 m and stops. Measure the actual
     distance travelled, then:
       python3 calibrate_odometry.py compute --mode drive \
           --odom-m <printed value> --actual-m <measured> --current-mult 1.0
     Put the result into left/right_wheel_radius_multiplier.

  2. ROTATION (wheel_separation multiplier)
     Align the robot heading with a visual reference (wall edge, grid line), then:
       python3 calibrate_odometry.py rotate --namespace robot_2 --turns 5
     The robot spins in place until odometry says 5 full turns and stops. If odometry
     were perfect it would face the reference again. Measure the leftover angle
     offset in degrees (positive = robot rotated PAST the mark in the spin
     direction, negative = stopped short), then:
       python3 calibrate_odometry.py compute --mode rotate \
           --odom-deg <printed value> --offset-deg <measured> --current-mult 1.0
     Put the result into wheel_separation_multiplier.

Run INSIDE the robot container (needs rclpy + DDS access to the robot's topics).
Topics used: /<namespace>/cmd_vel (twist_mux "general" input) and
/<namespace>/diffbot_base_controller/odom.

Repeat a run after applying a multiplier to verify: the residual should be near zero.
Ctrl-C at any moment stops the robot.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomIntegrator(Node):
    """Accumulates unwrapped yaw and path length from the controller odometry."""

    def __init__(self, namespace: str):
        super().__init__('odom_calibrator')
        self.accumulated_yaw = 0.0      # rad, signed, unwrapped
        self.accumulated_dist = 0.0     # m, path length
        self.received = False
        self._last_yaw = None
        self._last_xy = None

        self.cmd_pub = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)
        self.create_subscription(
            Odometry, f'/{namespace}/diffbot_base_controller/odom', self._odom_cb, 10
        )

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        if self._last_yaw is not None:
            dyaw = yaw - self._last_yaw
            # Unwrap across the +-pi boundary
            if dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            self.accumulated_yaw += dyaw
            self.accumulated_dist += math.hypot(p.x - self._last_xy[0], p.y - self._last_xy[1])

        self._last_yaw = yaw
        self._last_xy = (p.x, p.y)
        self.received = True

    def send(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

    def stop(self) -> None:
        self.send(0.0, 0.0)


def run_motion(node: OdomIntegrator, vx: float, wz: float,
               done, progress, timeout: float) -> None:
    """Stream cmd_vel at 20 Hz until done() or timeout; always stop the robot."""
    # Wait for first odom message
    t0 = time.monotonic()
    while not node.received:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t0 > 10.0:
            print('ERROR: no odometry received in 10 s — are the drivers running?')
            sys.exit(1)

    last_print = 0.0
    t0 = time.monotonic()
    try:
        while not done():
            if time.monotonic() - t0 > timeout:
                print('ERROR: timeout — stopping. Is the robot actually moving?')
                break
            node.send(vx, wz)
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_print > 1.0:
                print(f'  {progress()}', flush=True)
                last_print = now
    finally:
        # Stop firmly: a few zero commands, then settle so odom catches up
        for _ in range(10):
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.05)
    settle_until = time.monotonic() + 2.0
    while time.monotonic() < settle_until:
        rclpy.spin_once(node, timeout_sec=0.1)


def cmd_rotate(args) -> None:
    rclpy.init()
    node = OdomIntegrator(args.namespace)
    target_rad = args.turns * 2.0 * math.pi
    direction = 1.0 if args.turns > 0 else -1.0
    print(f'Rotating in place: {args.turns} odometry turns at {args.speed} rad/s')
    print('Robot heading must be aligned with your reference mark BEFORE this run!')
    run_motion(
        node,
        vx=0.0,
        wz=direction * abs(args.speed),
        done=lambda: abs(node.accumulated_yaw) >= abs(target_rad),
        progress=lambda: f'odom yaw: {math.degrees(node.accumulated_yaw):8.1f} deg',
        timeout=abs(target_rad / args.speed) * 3.0 + 10.0,
    )
    odom_deg = math.degrees(node.accumulated_yaw)
    print(f'\nDone. Odometry rotated: {odom_deg:.1f} deg')
    print('Measure the angular offset from your mark (deg, + = past the mark in spin')
    print('direction), then run:')
    print(f'  python3 calibrate_odometry.py compute --mode rotate '
          f'--odom-deg {odom_deg:.1f} --offset-deg <MEASURED> '
          f'--current-mult <current wheel_separation_multiplier>')
    node.destroy_node()
    rclpy.shutdown()


def cmd_drive(args) -> None:
    rclpy.init()
    node = OdomIntegrator(args.namespace)
    print(f'Driving straight: {args.distance} m by odometry at {args.speed} m/s')
    print('Robot must start at the tape-measure zero mark!')
    run_motion(
        node,
        vx=abs(args.speed),
        wz=0.0,
        done=lambda: node.accumulated_dist >= abs(args.distance),
        progress=lambda: f'odom dist: {node.accumulated_dist:6.3f} m',
        timeout=abs(args.distance / args.speed) * 3.0 + 10.0,
    )
    odom_m = node.accumulated_dist
    print(f'\nDone. Odometry travelled: {odom_m:.4f} m')
    print('Measure the actual distance with a tape, then run:')
    print(f'  python3 calibrate_odometry.py compute --mode drive '
          f'--odom-m {odom_m:.4f} --actual-m <MEASURED> '
          f'--current-mult <current wheel_radius_multiplier>')
    node.destroy_node()
    rclpy.shutdown()


def cmd_compute(args) -> None:
    if args.mode == 'rotate':
        if args.odom_deg is None or args.offset_deg is None:
            sys.exit('compute --mode rotate requires --odom-deg and --offset-deg')
        # Physical rotation = nearest whole number of turns + measured offset.
        whole_turns = round(args.odom_deg / 360.0)
        actual_deg = whole_turns * 360.0 + args.offset_deg
        new_mult = args.current_mult * args.odom_deg / actual_deg
        print(f'Odometry: {args.odom_deg:.1f} deg, physical: {actual_deg:.1f} deg '
              f'({whole_turns} turns {args.offset_deg:+.1f} deg)')
        print(f'\n  wheel_separation_multiplier: {args.current_mult} -> {new_mult:.4f}\n')
        print('Update it in diffdrive_jetbot/bringup/config/diffbot_controllers.yaml,')
        print('redeploy, and re-run "rotate" to verify (offset should be near 0).')
    else:  # drive
        if args.odom_m is None or args.actual_m is None:
            sys.exit('compute --mode drive requires --odom-m and --actual-m')
        new_mult = args.current_mult * args.actual_m / args.odom_m
        print(f'Odometry: {args.odom_m:.4f} m, physical: {args.actual_m:.4f} m')
        print(f'\n  left/right_wheel_radius_multiplier: {args.current_mult} -> {new_mult:.4f}\n')
        print('Update both wheel radius multipliers in diffbot_controllers.yaml,')
        print('redeploy, and re-run "drive" to verify.')


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest='command', required=True)

    p_rot = sub.add_parser('rotate', help='spin N odometry turns in place, then stop')
    p_rot.add_argument('--namespace', default='robot_2')
    p_rot.add_argument('--turns', type=float, default=5.0,
                       help='full odometry revolutions (negative = clockwise), default 5')
    p_rot.add_argument('--speed', type=float, default=0.4,
                       help='angular speed rad/s (keep low to avoid wheel slip), default 0.4')
    p_rot.set_defaults(func=cmd_rotate)

    p_drv = sub.add_parser('drive', help='drive N odometry meters straight, then stop')
    p_drv.add_argument('--namespace', default='robot_2')
    p_drv.add_argument('--distance', type=float, default=1.0,
                       help='odometry distance in meters, default 1.0')
    p_drv.add_argument('--speed', type=float, default=0.08,
                       help='linear speed m/s, default 0.08')
    p_drv.set_defaults(func=cmd_drive)

    p_cmp = sub.add_parser('compute', help='compute the corrected multiplier')
    p_cmp.add_argument('--mode', choices=['rotate', 'drive'], required=True)
    p_cmp.add_argument('--current-mult', type=float, default=1.0,
                       help='multiplier currently set in diffbot_controllers.yaml')
    p_cmp.add_argument('--odom-deg', type=float, help='odometry angle printed by "rotate"')
    p_cmp.add_argument('--offset-deg', type=float,
                       help='measured offset from the mark, deg (+ = past the mark)')
    p_cmp.add_argument('--odom-m', type=float, help='odometry distance printed by "drive"')
    p_cmp.add_argument('--actual-m', type=float, help='tape-measured distance, m')
    p_cmp.set_defaults(func=cmd_compute)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
