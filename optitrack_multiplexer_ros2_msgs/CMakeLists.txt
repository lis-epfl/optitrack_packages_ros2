cmake_minimum_required(VERSION 3.8)
project(optitrack_multiplexer_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(optitrack_wrapper_ros2_msgs REQUIRED)

set(MSG_FILES
  "msg/MarkerBase.msg"
  "msg/RigidBodyBase.msg"
  "msg/RigidBodyStamped.msg"
  "msg/SkeletonStamped.msg"
  "msg/UnlabeledMarkersStamped.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs optitrack_wrapper_ros2_msgs
)

ament_package()
