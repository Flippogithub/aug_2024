#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import math

class GoalRepublisher(Node):
    def __init__(self):
        super().__init__('goal_republisher')
        
        self.current_goal = None
        self.current_transform = None
        
        # Subscribe to goal and tf
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
            
        self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
            
        # Publishers
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/continuous_goal',
            10)
            
        self.goal_publisher_odom = self.create_publisher(
            PoseStamped,
            '/continuous_goal_odom',
            10)
            
        self.timer = self.create_timer(0.1, self.timer_callback)

    def goal_callback(self, msg):
        self.current_goal = msg
        self.get_logger().info(f'Received goal: x={msg.pose.position.x}, y={msg.pose.position.y}')

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                self.current_transform = transform
                break

    def timer_callback(self):
        if self.current_goal is not None:
            # Publish original goal
            self.goal_publisher.publish(self.current_goal)
            
            # If we have transform info, publish transformed goal
            if self.current_transform is not None:
                transformed_goal = PoseStamped()
                transformed_goal.header.stamp = self.get_clock().now().to_msg()
                transformed_goal.header.frame_id = 'odom'
                
                # Apply transform
                t = self.current_transform.transform.translation
                transformed_goal.pose.position.x = self.current_goal.pose.position.x - t.x
                transformed_goal.pose.position.y = self.current_goal.pose.position.y - t.y
                transformed_goal.pose.position.z = self.current_goal.pose.position.z - t.z
                
                # Copy orientation for now
                transformed_goal.pose.orientation = self.current_goal.pose.orientation
                
                self.goal_publisher_odom.publish(transformed_goal)
                self.get_logger().info(f'Published transformed goal: x={transformed_goal.pose.position.x}')

def main():
    rclpy.init()
    node = GoalRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()