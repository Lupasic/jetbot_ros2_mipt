#!/usr/bin/env python3
"""
Nav2 Status Publisher - monitors nav2 action status and publishes to simple topics.
Works on the robot to bypass Zenoh action callback issues.
"""

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import UInt8, String


class Nav2StatusPublisher(Node):
    """
    Monitors local nav2 action server status and publishes to Zenoh-friendly topics.
    Bridges between nav2 actions and simple topic communication.
    """
    
    def __init__(self):
        super().__init__('nav2_status_publisher')
        
        # Subscribe to nav2 action status topic (standard ROS2 action status)
        self.action_status_sub = self.create_subscription(
            GoalStatusArray,
            'navigate_to_pose/_action/status',
            self.on_action_status,
            10
        )
        
        # Publishers for simplified status (Zenoh-compatible topics)
        self.status_pub = self.create_publisher(
            UInt8,
            'nav2_goal_status',
            10
        )
        
        self.result_pub = self.create_publisher(
            String,
            'nav2_goal_result',
            10
        )
        
        # Track last status to avoid duplicate publications
        self.last_status = None
        self.last_goal_id = None
        
        self.get_logger().info("Nav2 Status Publisher initialized")
        self.get_logger().info("Subscribing to: navigate_to_pose/_action/status")
        self.get_logger().info("Publishing to: nav2_goal_status, nav2_goal_result")
    
    def on_action_status(self, msg: GoalStatusArray):
        """
        Process nav2 action status updates.
        
        Status codes (action_msgs/GoalStatus):
        - 0: STATUS_UNKNOWN
        - 1: STATUS_ACCEPTED
        - 2: STATUS_EXECUTING
        - 3: STATUS_CANCELING
        - 4: STATUS_SUCCEEDED
        - 5: STATUS_CANCELED
        - 6: STATUS_ABORTED
        """
        if not msg.status_list:
            return
        
        # Get the latest goal status (most recent entry)
        latest = msg.status_list[-1]
        status = latest.status
        goal_id = latest.goal_info.goal_id.uuid
        
        # Convert UUID bytes to string for comparison
        goal_id_str = ''.join(format(b, '02x') for b in goal_id)
        
        # Only publish if status or goal changed
        if status == self.last_status and goal_id_str == self.last_goal_id:
            return
        
        self.last_status = status
        self.last_goal_id = goal_id_str
        
        # Status name mapping for logging
        status_names = {
            0: "UNKNOWN",
            1: "ACCEPTED", 
            2: "EXECUTING",
            3: "CANCELING",
            4: "SUCCEEDED",
            5: "CANCELED",
            6: "ABORTED"
        }
        
        status_name = status_names.get(status, f"UNKNOWN_{status}")
        self.get_logger().info(f"Nav2 status update: {status_name} (code: {status})")
        
        # Publish numeric status
        status_msg = UInt8()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        
        # Publish final result for terminal statuses
        if status in [4, 5, 6]:  # SUCCEEDED, CANCELED, ABORTED
            result_msg = String()
            
            if status == 4:
                result_msg.data = "SUCCESS"
            elif status == 5:
                result_msg.data = "CANCELED"
            else:  # status == 6
                result_msg.data = "FAILED"
            
            self.result_pub.publish(result_msg)
            self.get_logger().info(f"Navigation completed: {result_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2StatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
