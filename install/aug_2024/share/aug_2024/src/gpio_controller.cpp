#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class GPIOController : public rclcpp::Node
{
public:
  GPIOController()
  : Node("gpio_controller")
  {
    // Declare parameters
    this->declare_parameter<int>("pin", 21);
    pin_ = this->get_parameter("pin").as_int();
    
    // Set up GPIO
    setup_gpio(pin_);
    
    // Create a subscriber
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "gpio_output_" + std::to_string(pin_), 10,
      std::bind(&GPIOController::gpio_callback, this, std::placeholders::_1));
  }
  
  ~GPIOController()
  {
    // Unexport GPIO pin when shutting down
    std::ofstream unexport_file("/sys/class/gpio/unexport");
    if (unexport_file.is_open()) {
      unexport_file << pin_;
      unexport_file.close();
    }
  }

private:
  void setup_gpio(int pin)
  {
    // Check if GPIO is already exported
    std::string gpio_path = "/sys/class/gpio/gpio" + std::to_string(pin);
    std::ifstream gpio_exists(gpio_path);
    
    // If not exported, export it
    if (!gpio_exists.good()) {
      std::ofstream export_file("/sys/class/gpio/export");
      if (export_file.is_open()) {
        export_file << pin;
        export_file.close();
      }
    }
    
    // Set direction to output
    std::ofstream direction_file(gpio_path + "/direction");
    if (direction_file.is_open()) {
      direction_file << "out";
      direction_file.close();
    }
  }
  
  void gpio_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // Write to GPIO value
    std::ofstream value_file("/sys/class/gpio/gpio" + std::to_string(pin_) + "/value");
    if (value_file.is_open()) {
      if (msg->data) {
        value_file << "1";  // Set HIGH
        RCLCPP_INFO(this->get_logger(), "Setting pin %d HIGH", pin_);
      } else {
        value_file << "0";  // Set LOW
        RCLCPP_INFO(this->get_logger(), "Setting pin %d LOW", pin_);
      }
      value_file.close();
    }
  }
  
  int pin_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPIOController>());
  rclcpp::shutdown();
  return 0;
}