#!/usr/bin/env python3
"""
AMCL Pose Relay - receives initial pose through network and republishes locally.
Works on the robot to ensure proper timing for AMCL initialization.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener


class AmclPoseRelay(Node):
    """
    Relay node that receives initial pose from external source (through Zenoh)
    and republishes it locally with synced timestamp for AMCL.
    """
    
    def __init__(self):
        super().__init__('amcl_pose_relay')
        
        # TF buffer and listener for checking transform availability
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Parameters
        self.declare_parameter('tf_timeout', 2.0)
        self.tf_timeout = self.get_parameter('tf_timeout').value
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose_external',
            self.pose_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10)
        
        self.get_logger().info('AMCL Pose Relay initialized')
        self.get_logger().info(f'TF timeout: {self.tf_timeout}s')
    
    def pose_callback(self, msg):
        """
        Receive initial pose and republish with synced timestamp.
        Waits for TF to be available before publishing.
        Uses base_footprint -> odom transform (same as AMCL needs).
        """
        try:
            self.get_logger().info('Received initial pose, checking TF availability...')
            
            # Extract namespace from frame_id (e.g., 'robot_2/map' -> 'robot_2')
            map_frame = msg.header.frame_id  # e.g., 'robot_2/map'
            
            if '/' in map_frame:
                namespace = map_frame.rsplit('/', 1)[0]
                # Build frame names with namespace
                base_frame = f'{namespace}/base_footprint'
                odom_frame = f'{namespace}/odom'
            else:
                # No namespace case
                base_frame = 'base_footprint'
                odom_frame = 'odom'
            
            self.get_logger().info(f'Looking for TF: {base_frame} -> {odom_frame}')
            
            # Wait for transform to be available (same transform AMCL needs)
            timeout_duration = Duration(seconds=self.tf_timeout)
            
            if self.tf_buffer.can_transform(
                odom_frame,
                base_frame,
                Time(),  # Latest available time
                timeout=timeout_duration
            ):
                # Get the latest transform to sync timestamp
                transform = self.tf_buffer.lookup_transform(
                    odom_frame,
                    base_frame,
                    Time(),  # Use latest available
                    timeout=timeout_duration
                )
                
                # Create relayed message with synced timestamp
                relayed_msg = PoseWithCovarianceStamped()
                relayed_msg.header.frame_id = msg.header.frame_id
                relayed_msg.header.stamp = transform.header.stamp
                relayed_msg.pose = msg.pose
                
                self.publisher_.publish(relayed_msg)
                
                self.get_logger().info(
                    f"Relayed initial pose: x={msg.pose.pose.position.x:.3f}, "
                    f"y={msg.pose.pose.position.y:.3f}, "
                    f"frame='{msg.header.frame_id}', "
                    f"timestamp={transform.header.stamp.sec}.{transform.header.stamp.nanosec}"
                )
            else:
                self.get_logger().warn(
                    f'TF not available: {base_frame} -> {odom_frame} within {self.tf_timeout}s'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in pose relay: {str(e)}')


def main(args=None):
    """Main entry point for the AMCL pose relay node."""
    rclpy.init(args=args)
    node = AmclPoseRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AMCL Pose Relay shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
