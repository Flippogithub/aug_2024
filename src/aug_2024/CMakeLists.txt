cmake_minimum_required(VERSION 3.5)
project(aug_2024)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the path to your ros2_control_ws
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/ros2_control_ws/install")
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(diff_drive_controller REQUIRED)
#find_package(gazebo_ros REQUIRED)
#find_package(gazebo_ros2_control REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(BMC_2024 REQUIRED)
find_package(sllidar_ros2 REQUIRED)

# install
install(
  DIRECTORY config description launch worlds src
   
  DESTINATION
    share/${PROJECT_NAME}/
)
# Install Python scripts
# Install Python scripts with correct path
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/image_annotator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Add the GPIO controller executable
add_executable(gpio_controller src/gpio_controller.cpp)
ament_target_dependencies(gpio_controller rclcpp std_msgs)

# Install the executable
install(TARGETS
  gpio_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()