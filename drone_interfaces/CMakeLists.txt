cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

# ROS 2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 액션 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
