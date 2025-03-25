#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus as ActionGoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
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
        self.current_goal_handle = None
        self.navigation_attempts = 0
        self.MAX_ATTEMPTS = 3  # Maximum attempts for a single waypoint
        
        # Start navigation
        self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints completed')
            return

        # Reset navigation attempts for this waypoint
        self.navigation_attempts = 0
        
        self.get_logger().info(f'Preparing to navigate to waypoint {self.current_waypoint}')
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        wp = self.waypoints[self.current_waypoint]
        goal.pose.pose.position.x = wp['position']['x']
        goal.pose.pose.position.y = wp['position']['y']
        goal.pose.pose.position.z = wp['position']['z']
        goal.pose.pose.orientation.w = wp['orientation']['w']
        
        if 'x' in wp['orientation']:
            goal.pose.pose.orientation.x = wp['orientation']['x']
        if 'y' in wp['orientation']:
            goal.pose.pose.orientation.y = wp['orientation']['y']
        if 'z' in wp['orientation']:
            goal.pose.pose.orientation.z = wp['orientation']['z']
        
        self.get_logger().warn(f'DETAILED: Navigating to waypoint {self.current_waypoint} at position ({wp["position"]["x"]:.2f}, {wp["position"]["y"]:.2f})')
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error(f'Goal to waypoint {self.current_waypoint} REJECTED')
                
                # Increment attempts and handle failure
                self.navigation_attempts += 1
                if self.navigation_attempts < self.MAX_ATTEMPTS:
                    self.get_logger().warn(f'Retrying waypoint {self.current_waypoint}')
                    self.navigate_to_next_waypoint()
                else:
                    self.get_logger().error(f'FAILED after {self.MAX_ATTEMPTS} attempts at waypoint {self.current_waypoint}')
                    self.current_waypoint += 1
                    self.navigate_to_next_waypoint()
                return

            self.get_logger().info('Goal accepted')
            self.current_goal_handle = goal_handle
            
            # Request result
            goal_handle.get_result_async().add_done_callback(self.get_result_callback)
        
        except Exception as e:
            self.get_logger().error(f'Error in goal response: {str(e)}')

    def get_result_callback(self, future):
        try:
            result = future.result()
            status = result.status

            # Detailed status logging
            status_map = {
                ActionGoalStatus.STATUS_SUCCEEDED: "Succeeded",
                ActionGoalStatus.STATUS_CANCELED: "Canceled",
                ActionGoalStatus.STATUS_ABORTED: "Aborted"
            }
            
            self.get_logger().warn(f'NAVIGATION RESULT for waypoint {self.current_waypoint}: {status_map.get(status, "Unknown")}')

            # Increment attempts
            self.navigation_attempts += 1

            if status == ActionGoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Reached waypoint {self.current_waypoint} successfully')
                self.current_waypoint += 1
                self.current_goal_handle = None
                
                # Immediately proceed to next waypoint
                if self.current_waypoint < len(self.waypoints):
                    self.navigate_to_next_waypoint()
                else:
                    self.get_logger().info('All waypoints completed')
            else:
                # Retry or move to next waypoint
                if self.navigation_attempts < self.MAX_ATTEMPTS:
                    self.get_logger().warn(f'Retrying waypoint {self.current_waypoint}')
                    self.navigate_to_next_waypoint()
                else:
                    self.get_logger().error(f'FAILED after {self.MAX_ATTEMPTS} attempts at waypoint {self.current_waypoint}')
                    self.current_waypoint += 1
                    self.navigate_to_next_waypoint()
        
        except Exception as e:
            self.get_logger().error(f'Error in result callback: {str(e)}')

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