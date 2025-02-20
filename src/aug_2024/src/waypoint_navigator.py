#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Wait for action server
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigation action server...')
            
        self.waypoints = [
           {'position': {'x': 2.15, 'y': 4.96, 'z': 0.0}, 
              'orientation': {'w': 0.7071, 'x': 0.0, 'y': 0.0, 'z': -0.7071}},
            {'position': {'x': 10.14, 'y': 3.16, 'z': 0.0}, 
              'orientation': {'w': 0.7071, 'x': 0.0, 'y': 0.0, 'z': 0.7071}},
            {'position': {'x': 10.14, 'y': 3.16, 'z': 0.0},
              'orientation': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 1.0}},
            {'position': {'x': 8.82, 'y': -7.15, 'z': 0.0}, 'orientation': {'w': 1.0}},
            {'position': {'x': -15.36, 'y': -6.84, 'z': 0.0}, 
              'orientation': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 1.0}},
            {'position': {'x': -11.956, 'y': 7.51, 'z': 0.0},
              'orientation': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': -1.0}},
        ]
        self.current_waypoint = 0
        self.timer = self.create_timer(5.0, self.navigate_to_next_waypoint)
        self.current_goal_handle = None

    def navigate_to_next_waypoint(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints completed')
            self.timer.cancel()
            return

        # Cancel any existing goals
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling previous goal...')
            self.current_goal_handle.cancel_goal_async()

        self.get_logger().info('Sending new goal...')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        wp = self.waypoints[self.current_waypoint]
        goal.pose.pose.position.x = wp['position']['x']
        goal.pose.pose.position.y = wp['position']['y']
        goal.pose.pose.position.z = wp['position']['z']
        goal.pose.pose.orientation.w = wp['orientation']['w']
        
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint} at position ({wp["position"]["x"]:.2f}, {wp["position"]["y"]:.2f})')
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.current_goal_handle = goal_handle
        
        # Request result
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached waypoint successfully')
            self.current_waypoint += 1
            self.current_goal_handle = None
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

def main():
    rclpy.init()
    try:
        node = WaypointNavigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()