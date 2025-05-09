import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os

class GPIOController(Node):
    def __init__(self):
        super().__init__('gpio_controller')
        self.pin = self.declare_parameter('pin', 21).value
        
        # Set up GPIO
        self.setup_gpio(self.pin)
        
        # Create a subscriber
        self.subscription = self.create_subscription(
            Bool,
            f'gpio_output_{self.pin}',
            self.gpio_callback,
            10)
    
    def setup_gpio(self, pin):
        # Export the GPIO pin
        if not os.path.exists(f'/sys/class/gpio/gpio{pin}'):
            with open('/sys/class/gpio/export', 'w') as f:
                f.write(str(pin))
        
        # Set direction to output
        with open(f'/sys/class/gpio/gpio{pin}/direction', 'w') as f:
            f.write('out')
    
    def gpio_callback(self, msg):
        # Write to GPIO value
        with open(f'/sys/class/gpio/gpio{self.pin}/value', 'w') as f:
            if msg.data:
                f.write('1')  # Set HIGH
                self.get_logger().info(f'Setting pin {self.pin} HIGH')
            else:
                f.write('0')  # Set LOW
                self.get_logger().info(f'Setting pin {self.pin} LOW')
    
    def destroy_node(self):
        # Unexport GPIO pin when shutting down
        with open('/sys/class/gpio/unexport', 'w') as f:
            f.write(str(self.pin))
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gpio_controller = GPIOController()
    rclpy.spin(gpio_controller)
    gpio_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()