cmake_minimum_required(VERSION 3.8)
project(onrobot_rg2ft_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/OnRobotRG2FTDriver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
