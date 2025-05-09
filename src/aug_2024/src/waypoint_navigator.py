#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus as ActionGoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import os  # For direct GPIO access

# Add this class for GPIO control
class GPIOController:
    def __init__(self, node):
        self.node = node
        self.controlled_pins = {}
    
    def setup_pin(self, pin, direction='out'):
        """Set up a GPIO pin with the specified direction."""
        # Check if pin is already exported
        if pin in self.controlled_pins:
            return
            
        gpio_path = f"/sys/class/gpio/gpio{pin}"
        if not os.path.exists(gpio_path):
            # Export the pin
            try:
                with open('/sys/class/gpio/export', 'w') as f:
                    f.write(str(pin))
                self.node.get_logger().info(f'Exported GPIO pin {pin}')
            except Exception as e:
                self.node.get_logger().error(f'Failed to export GPIO pin {pin}: {str(e)}')
                return False
        
        # Set the direction
        try:
            with open(f'{gpio_path}/direction', 'w') as f:
                f.write(direction)
            self.node.get_logger().info(f'Set GPIO pin {pin} direction to {direction}')
            self.controlled_pins[pin] = direction
            return True
        except Exception as e:
            self.node.get_logger().error(f'Failed to set GPIO pin {pin} direction: {str(e)}')
            return False
    
    def set_pin_value(self, pin, value):
        """Set a GPIO pin to the specified value (1/0)."""
        if pin not in self.controlled_pins:
            if not self.setup_pin(pin):
                return False
        
        try:
            with open(f'/sys/class/gpio/gpio{pin}/value', 'w') as f:
                f.write(str(value))
            self.node.get_logger().info(f'Set GPIO pin {pin} to {value}')
            return True
        except Exception as e:
            self.node.get_logger().error(f'Failed to set GPIO pin {pin} value: {str(e)}')
            return False
    
    def cleanup(self):
        """Unexport all controlled pins."""
        for pin in list(self.controlled_pins.keys()):
            try:
                with open('/sys/class/gpio/unexport', 'w') as f:
                    f.write(str(pin))
                self.node.get_logger().info(f'Unexported GPIO pin {pin}')
                del self.controlled_pins[pin]
            except Exception as e:
                self.node.get_logger().error(f'Failed to unexport GPIO pin {pin}: {str(e)}')


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Initialize the GPIO controller
        self.gpio_controller = GPIOController(self)
        
        # Set up pins you want to control
        self.gpio_pins = {
            'led': 21,       # Example: LED on pin 21
            'motor': 22,     # Example: Motor control on pin 22
            'sensor': 23     # Example: Sensor control on pin 23
        }
        
        # Initialize GPIO pins
        for name, pin in self.gpio_pins.items():
            self.gpio_controller.setup_pin(pin, 'out')
        
        # Define GPIO actions for specific waypoints
        # Format: {waypoint_index: [(pin_name, value), ...]}
        self.waypoint_gpio_actions = {
            1: [('led', 1)],                        # Turn on LED at waypoint 1
            2: [('motor', 1), ('sensor', 1)],       # Turn on motor and sensor at waypoint 2
            4: [('led', 0), ('motor', 0)],          # Turn off LED and motor at waypoint 4
            5: [('sensor', 0)]                      # Turn off sensor at waypoint 5
        }
        
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
                
                # Perform GPIO actions for this waypoint
                self.perform_gpio_actions()
                
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
    
    def perform_gpio_actions(self):
        """Perform GPIO actions associated with the current waypoint."""
        if self.current_waypoint in self.waypoint_gpio_actions:
            actions = self.waypoint_gpio_actions[self.current_waypoint]
            self.get_logger().info(f'Performing GPIO actions for waypoint {self.current_waypoint}')
            
            for pin_name, value in actions:
                if pin_name in self.gpio_pins:
                    pin = self.gpio_pins[pin_name]
                    success = self.gpio_controller.set_pin_value(pin, value)
                    if success:
                        self.get_logger().info(f'GPIO {pin_name} (pin {pin}) set to {value}')
                    else:
                        self.get_logger().error(f'Failed to set GPIO {pin_name} (pin {pin}) to {value}')

    def destroy_node(self):
        # Clean up GPIO pins when the node is destroyed
        self.gpio_controller.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    try:
        node = WaypointNavigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure to clean up
        if 'node' in locals():
            node.destroy_node()
        rclpy.