cmake_minimum_required(VERSION 3.8)
project(onrobot_rg2ft_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/RG2FTCommand.msg"
  "msg/RG2FTState.msg"
)

set(srv_files
  "srv/SetProximityOffsets.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()


