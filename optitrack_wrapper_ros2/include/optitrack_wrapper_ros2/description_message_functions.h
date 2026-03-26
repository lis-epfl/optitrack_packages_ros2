#ifndef DESCRIPTION_MESSAGE_FUNCTIONS_H_
#define DESCRIPTION_MESSAGE_FUNCTIONS_H_

#include "optitrack_wrapper_ros2_msgs/msg/camera_description.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/device_description.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/force_plate_description.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/marker_set_description.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/rigid_body_description.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/skeleton_description.hpp"

#include <NatNetTypes.h>

namespace optitrack_wrapper {

// get marker set description as a ros msg
::optitrack_wrapper_ros2_msgs::msg::MarkerSetDescription
GetMarkerSetDescriptionMessage(sMarkerSetDescription *p_ms);

// get rigid body description as a ros msg
::optitrack_wrapper_ros2_msgs::msg::RigidBodyDescription
GetRigidBodyDescriptionMessage(sRigidBodyDescription *p_rb);

// get skeleton description as a ros msg
::optitrack_wrapper_ros2_msgs::msg::SkeletonDescription
GetSkeletonDescriptionMessage(sSkeletonDescription *p_sk);

// get force plate description as a ros msg
::optitrack_wrapper_ros2_msgs::msg::ForcePlateDescription
GetForcePlateDescriptionMessage(sForcePlateDescription *p_fp);

// get device description as a ros msg
::optitrack_wrapper_ros2_msgs::msg::DeviceDescription
GetDeviceDescriptionMessage(sDeviceDescription *p_device);

// get device description as a ros msg
::optitrack_wrapper_ros2_msgs::msg::CameraDescription
GetCameraDescriptionMessage(sCameraDescription *p_camera);

} // namespace optitrack_wrapper

#endif
