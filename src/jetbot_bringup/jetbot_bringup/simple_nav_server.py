#!/usr/bin/env python3
"""
SimpleNavServer - Topic-based wrapper for Nav2 NavigateToPose action.

Subscribes to nav/goal (PoseStamped) and publishes nav/status (GoalStatus).
Internally manages Nav2 action client locally on the robot.

This design works reliably through Zenoh bridge by avoiding action communication
across the bridge - only simple topics are bridged.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty


class SimpleNavServer(Node):
    """
    Simple navigation server that bridges topic-based commands to Nav2 actions.
    
    Topics:
        Subscribed:
            - nav/goal (PoseStamped): Navigation goal pose
        Published:
            - nav/status (GoalStatus): Current navigation status
    
    Action Clients:
        - /navigate_to_pose: Nav2 navigation action (local, not bridged)
    """
    
    def __init__(self):
        super().__init__('simple_nav_server')
        
        # Current goal tracking
        self._goal_handle = None
        self._current_status = GoalStatus.STATUS_UNKNOWN
        
        # Subscribe to goal commands
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'nav/goal',
            self._goal_callback,
            10
        )
        
        # Subscribe to cancel commands
        self.cancel_sub = self.create_subscription(
            Empty,
            'nav/cancel',
            self._cancel_callback,
            10
        )
        
        # Publisher for status updates
        self.status_pub = self.create_publisher(
            GoalStatus,
            'nav/status',
            10
        )
        
        # Nav2 action client (local communication, not through Zenoh)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Timer for periodic status publishing
        self.status_timer = self.create_timer(
            0.1,  # Publish status at 10 Hz
            self._publish_status
        )
        
        self.get_logger().info('SimpleNavServer initialized, waiting for goals on nav/goal')
        self.get_logger().info('Publishing status on nav/status')
        
        # Publish initial UNKNOWN status to clear any old statuses
        self._publish_status()
        
        # Wait for Nav2 action server
        self._wait_for_nav2()
    
    def _wait_for_nav2(self):
        """Wait for Nav2 action server to become available."""
        self.get_logger().info('Waiting for Nav2 action server...')
        if self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Nav2 action server available')
        else:
            self.get_logger().warn('Nav2 action server not available after 10s, will retry on goal')
    
    def _goal_callback(self, msg: PoseStamped):
        """
        Handle incoming goal from topic.
        
        Args:
            msg: PoseStamped with target pose
        """
        self.get_logger().info(
            f'Received goal: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}, '
            f'frame={msg.header.frame_id}'
        )
        
        # Cancel any existing goal
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling previous goal')
            self._cancel_current_goal()
            # Wait a bit for cancellation to complete
            import time
            time.sleep(0.1)
        
        # Reset status to unknown before sending new goal
        self._current_status = GoalStatus.STATUS_UNKNOWN
        
        # Send new goal to Nav2
        self._send_nav2_goal(msg)
    
    def _cancel_callback(self, msg: Empty):
        """
        Handle cancel request from topic.
        
        Args:
            msg: Empty message
        """
        self.get_logger().info('Received cancel request')
        if self._goal_handle is not None:
            self._cancel_current_goal()
        else:
            self.get_logger().info('No active goal to cancel')
    
    def _send_nav2_goal(self, pose: PoseStamped):
        """
        Send goal to Nav2 action server.
        
        Args:
            pose: Target pose
        """
        # Wait for action server if not ready
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Nav2 action server not ready, waiting...')
            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Nav2 action server unavailable, cannot send goal')
                self._current_status = GoalStatus.STATUS_ABORTED
                self._publish_status()
                return
        
        # Create Nav2 goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Update status to accepted
        self._current_status = GoalStatus.STATUS_ACCEPTED
        self._publish_status()
        
        # Send goal asynchronously
        self.get_logger().info('Sending goal to Nav2 action server')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        """
        Handle goal acceptance/rejection from Nav2.
        
        Args:
            future: Future containing goal handle
        """
        self._goal_handle = future.result()
        
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self._current_status = GoalStatus.STATUS_ABORTED
            self._publish_status()
            return
        
        self.get_logger().info('Goal accepted by Nav2')
        self._current_status = GoalStatus.STATUS_EXECUTING
        self._publish_status()
        
        # Wait for result
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)
    
    def _feedback_callback(self, feedback_msg):
        """
        Handle feedback from Nav2 (optional, for logging).
        
        Args:
            feedback_msg: Feedback from NavigateToPose action
        """
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Nav2 feedback - distance remaining: {feedback.distance_remaining:.2f}m',
            throttle_duration_sec=2.0
        )
    
    def _get_result_callback(self, future):
        """
        Handle final result from Nav2.
        
        Args:
            future: Future containing result
        """
        self.get_logger().info('_get_result_callback CALLED')
        try:
            result = future.result()
            status = result.status
            
            self.get_logger().info(f'Nav2 result received with status: {status}')
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation succeeded')
                self._current_status = GoalStatus.STATUS_SUCCEEDED
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn('Navigation aborted')
                self._current_status = GoalStatus.STATUS_ABORTED
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Navigation canceled')
                self._current_status = GoalStatus.STATUS_CANCELED
            else:
                self.get_logger().warn(f'Navigation ended with status: {status}')
                self._current_status = status
            
            self._publish_status()
            self.get_logger().info(f'Published final status: {self._current_status}')
            self._goal_handle = None
            
        except Exception as e:
            self.get_logger().error(f'Error getting result: {e}')
            self._current_status = GoalStatus.STATUS_ABORTED
            self._publish_status()
            self._goal_handle = None
    
    def _cancel_current_goal(self):
        """Cancel currently executing goal."""
        if self._goal_handle is not None:
            try:
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda f: self.get_logger().info('Goal cancellation completed')
                )
            except Exception as e:
                self.get_logger().error(f'Error cancelling goal: {e}')
    
    def _publish_status(self):
        """Publish current navigation status."""
        status_msg = GoalStatus()
        status_msg.status = self._current_status
        # Add timestamp from robot's clock
        status_msg.goal_info.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(status_msg)
        
        # Log status changes (not every publish)
        if not hasattr(self, '_last_logged_status') or self._last_logged_status != self._current_status:
            status_names = {
                GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
                GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
                GoalStatus.STATUS_EXECUTING: 'EXECUTING',
                GoalStatus.STATUS_CANCELING: 'CANCELING',
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_CANCELED: 'CANCELED',
                GoalStatus.STATUS_ABORTED: 'ABORTED'
            }
            status_name = status_names.get(self._current_status, f'UNKNOWN({self._current_status})')
            self.get_logger().info(f'Status: {status_name}')
            self._last_logged_status = self._current_status


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    server = SimpleNavServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
