cmake_minimum_required(VERSION 3.8)
project(rover_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoardStatus.msg"
  "msg/BaseCommand.msg"
  "msg/BaseState.msg"
  "msg/DriveMode.msg"
  "msg/ArmCommand.msg"
  "msg/ArmState.msg"
  "msg/MastCommand.msg"
  "msg/MastState.msg"
  "msg/MicroscopeCommand.msg"
  "msg/MicroscopeState.msg"
  "msg/AntennaCommand.msg"
  "msg/AntennaState.msg"
  "msg/PowerState.msg"
  "srv/HomeJoint.srv"
  "srv/EmergencyStop.srv"
  "srv/ArmPreset.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
