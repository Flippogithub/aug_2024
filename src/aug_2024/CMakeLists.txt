cmake_minimum_required(VERSION 3.8)
project(aug_2024)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# Uncomment and modify if you have executable targets
#add_executable(skunk_aug src/skunk_aug.cpp)
#target_include_directories(skunk_aug PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(skunk_aug PUBLIC c_std_99 cxx_std_17)
#ament_target_dependencies(skunk_aug rclcpp std_msgs)

#install(TARGETS skunk_aug
#  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files, config files, and URDF files
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install additional folders if they exist and are needed
install(
  DIRECTORY
    meshes
    urdf
    models
    scripts
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Generate URDF file from xacro
xacro_add_files(
  description/urdf/aug_2024.xacro
  TARGET aug_2024_generated_urdf
  INSTALL DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()