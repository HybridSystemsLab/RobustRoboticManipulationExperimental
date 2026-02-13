cmake_minimum_required(VERSION 3.5)
project(pns_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(elfin_robot_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(elfin_ethercat_driver)
find_package(pluginlib REQUIRED REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(
  PROGRAMS
  scripts/cmd_move_server.py
  DESTINATION
  lib/${PROJECT_NAME})

ament_package()