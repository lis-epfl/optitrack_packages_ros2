#include "description_message_functions.h"

namespace optitrack_wrapper {

::optitrack_wrapper_ros2_msgs::msg::MarkerSetDescription
GetMarkerSetDescriptionMessage(sMarkerSetDescription *p_ms) {

  ::optitrack_wrapper_ros2_msgs::msg::MarkerSetDescription
      marker_set_description;
  marker_set_description.name = p_ms->szName;
  marker_set_description.n_markers = p_ms->nMarkers;
  for (int i = 0; i < p_ms->nMarkers; i++)
    marker_set_description.marker_names.push_back(p_ms->szMarkerNames[i]);
  return marker_set_description;
}

::optitrack_wrapper_ros2_msgs::msg::RigidBodyDescription
GetRigidBodyDescriptionMessage(sRigidBodyDescription *p_rb) {

  ::optitrack_wrapper_ros2_msgs::msg::RigidBodyDescription
      rigid_body_description;
  rigid_body_description.name = p_rb->szName;
  rigid_body_description.id = p_rb->ID;
  rigid_body_description.parent_id = p_rb->parentID;
  rigid_body_description.offset.x = p_rb->offsetx;
  rigid_body_description.offset.y = p_rb->offsety;
  rigid_body_description.offset.z = p_rb->offsetz;

  rigid_body_description.n_markers = p_rb->nMarkers;

  if (p_rb->MarkerPositions != NULL && p_rb->MarkerRequiredLabels != NULL) {
    for (int marker_idx = 0; marker_idx < p_rb->nMarkers; ++marker_idx) {
      // create position for marker
      geometry_msgs::msg::Point marker_position;
      marker_position.x = p_rb->MarkerPositions[marker_idx][0];
      marker_position.y = p_rb->MarkerPositions[marker_idx][1];
      marker_position.z = p_rb->MarkerPositions[marker_idx][2];

      rigid_body_description.marker_positions.push_back(marker_position);
      rigid_body_description.marker_required_labels.push_back(
          p_rb->MarkerRequiredLabels[marker_idx]);
      // Something wrong with the SDK it doesn't get the names
      /* rigid_body_description.marker_names.push_back( */
      /*     p_rb->szMarkerNames[marker_idx]); */
    }
  }
  return rigid_body_description;
}

::optitrack_wrapper_ros2_msgs::msg::SkeletonDescription
GetSkeletonDescriptionMessage(sSkeletonDescription *p_sk) {

  ::optitrack_wrapper_ros2_msgs::msg::SkeletonDescription skeleton_description;
  skeleton_description.name = p_sk->szName;
  skeleton_description.id = p_sk->skeletonID;
  skeleton_description.n_rigid_bodies = p_sk->nRigidBodies;
  for (int j = 0; j < p_sk->nRigidBodies; j++) {
    sRigidBodyDescription *p_rb = &p_sk->RigidBodies[j];
    ::optitrack_wrapper_ros2_msgs::msg::RigidBodyDescription
        rigid_body_description = GetRigidBodyDescriptionMessage(p_rb);
    skeleton_description.rigid_body_descriptions.push_back(
        rigid_body_description);
  }
  return skeleton_description;
}

::optitrack_wrapper_ros2_msgs::msg::ForcePlateDescription
GetForcePlateDescriptionMessage(sForcePlateDescription *p_fp) {

  ::optitrack_wrapper_ros2_msgs::msg::ForcePlateDescription
      force_plate_description;

  force_plate_description.id = p_fp->ID;
  force_plate_description.serial_no = p_fp->strSerialNo;
  force_plate_description.width = p_fp->fWidth;
  force_plate_description.length = p_fp->fLength;
  force_plate_description.origin.x = p_fp->fOriginX;
  force_plate_description.origin.y = p_fp->fOriginY;
  force_plate_description.origin.z = p_fp->fOriginZ;

  for (int i = 0; i < 12; i++) {
    for (int j = 0; j < 12; j++) {
      force_plate_description.cal_mat[i * 12 + j] = p_fp->fCalMat[i][j];
    }
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      force_plate_description.corners[i * 3 + j] = p_fp->fCorners[i][j];
    }
  }

  force_plate_description.plate_type = p_fp->iPlateType;
  force_plate_description.channel_data_type = p_fp->iChannelDataType;
  force_plate_description.n_channels = p_fp->nChannels;
  for (int i_channel = 0; i_channel < p_fp->nChannels; i_channel++)
    force_plate_description.channel_names.push_back(
        p_fp->szChannelNames[i_channel]);
  return force_plate_description;
}

::optitrack_wrapper_ros2_msgs::msg::DeviceDescription
GetDeviceDescriptionMessage(sDeviceDescription *p_device) {

  ::optitrack_wrapper_ros2_msgs::msg::DeviceDescription device_description;
  device_description.name = p_device->strName;
  device_description.serial_no = p_device->strSerialNo;
  device_description.id = p_device->ID;
  device_description.device_type = p_device->iDeviceType;
  device_description.channel_data_type = p_device->iChannelDataType;
  device_description.n_channels = p_device->nChannels;
  for (int i_channel = 0; i_channel < p_device->nChannels; i_channel++)
    device_description.channel_names.push_back(
        p_device->szChannelNames[i_channel]);

  return device_description;
}

::optitrack_wrapper_ros2_msgs::msg::CameraDescription
GetCameraDescriptionMessage(sCameraDescription *p_camera) {

  ::optitrack_wrapper_ros2_msgs::msg::CameraDescription camera_description;

  camera_description.name = p_camera->strName;
  camera_description.pose.position.x = p_camera->x;
  camera_description.pose.position.y = p_camera->y;
  camera_description.pose.position.z = p_camera->z;
  camera_description.pose.orientation.x = p_camera->qx;
  camera_description.pose.orientation.y = p_camera->qy;
  camera_description.pose.orientation.z = p_camera->qz;
  camera_description.pose.orientation.w = p_camera->qw;
  return camera_description;
}
} // namespace optitrack_wrapper
