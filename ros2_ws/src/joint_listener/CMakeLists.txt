cmake_minimum_required(VERSION 3.5)
project(joint_listener)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define custom messages and services with ROS 2 interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Robotiq2FGripper_robot_input.msg"
  "msg/Robotiq2FGripper_robot_output.msg"
  # You can list additional message or service files here
)

ament_package()

