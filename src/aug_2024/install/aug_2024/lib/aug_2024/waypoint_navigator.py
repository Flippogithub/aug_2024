#!/usr/bin/env python3  # Tells system to use Python 3 interpreter
import rclpy  # Import ROS2 Python library
from rclpy.node import Node  # For creating ROS2 nodes
from geometry_msgs.msg import PoseStamped  # Message type for robot poses
from nav2_msgs.action import NavigateToPose  # Nav2 action for navigation
from rclpy.action import ActionClient  # For sending action requests
from action_msgs.msg import GoalStatus  # To check navigation status

class WaypointNavigator(Node):  # Create class that inherits from ROS2 Node
   def __init__(self):  # Constructor function
       super().__init__('waypoint_navigator')  # Initialize parent Node class
       self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')  # Create client to send nav commands
       
       # List of waypoint coordinates
       self.waypoints = [
           {'position': {'x': 1.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'w': 1.0}},
           {'position': {'x': 1.0, 'y': 1.0, 'z': 0.0}, 'orientation': {'w': 1.0}},
           {'position': {'x': 0.0, 'y': 1.0, 'z': 0.0}, 'orientation': {'w': 1.0}}
       ]
       self.current_waypoint = 0  # Track which waypoint we're on

   async def navigate(self):  # Function that runs navigation sequence
       while self.current_waypoint < len(self.waypoints):  # Loop through waypoints
           goal = NavigateToPose.Goal()  # Create new navigation goal
           goal.pose.header.frame_id = 'map'  # Set reference frame
           # Set goal position from waypoint list
           goal.pose.pose.position.x = self.waypoints[self.current_waypoint]['position']['x']
           goal.pose.pose.position.y = self.waypoints[self.current_waypoint]['position']['y']
           goal.pose.pose.orientation.w = self.waypoints[self.current_waypoint]['orientation']['w']

           # Log status and send goal to Nav2
           self.get_logger().info(f'Navigating to waypoint {self.current_waypoint}')
           result = await self.nav_client.send_goal_async(goal)
           status = await result.get_result_async()  # Wait for result
           
           # Check if navigation succeeded
           if status.result.status == GoalStatus.STATUS_SUCCEEDED:
               self.get_logger().info('Reached waypoint')
               self.current_waypoint += 1  # Move to next waypoint
           else:
               self.get_logger().error('Navigation failed')
               break

def main():  # Main function to run the node
   rclpy.init()  # Initialize ROS2
   navigator = WaypointNavigator()  # Create node instance
   rclpy.spin(navigator)  # Keep node running
   rclpy.shutdown()  # Clean shutdown when done

if __name__ == '__main__':  # Python's way of running main() when script starts
   main()