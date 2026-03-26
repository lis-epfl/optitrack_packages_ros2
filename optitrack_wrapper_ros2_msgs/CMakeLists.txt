cmake_minimum_required(VERSION 3.8)
project(optitrack_wrapper_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

set(MSG_FILES
  "msg/AnalogChannelData.msg"
  "msg/CameraDescription.msg"
  "msg/DataDescriptions.msg"
  "msg/DeviceData.msg"
  "msg/DeviceDescription.msg"
  "msg/ForcePlateData.msg"
  "msg/ForcePlateDescription.msg"
  "msg/FrameOfMocapData.msg"
  "msg/Marker.msg"
  "msg/MarkerSetData.msg"
  "msg/MarkerSetDescription.msg"
  "msg/RigidBodyData.msg"
  "msg/RigidBodyDescription.msg"
  "msg/ServerDescription.msg"
  "msg/SkeletonData.msg"
  "msg/SkeletonDescription.msg"
)

set(SRV_FILES
  "srv/GetServerDescription.srv"
  "srv/GetDataDescriptions.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces geometry_msgs
 )

ament_package()
