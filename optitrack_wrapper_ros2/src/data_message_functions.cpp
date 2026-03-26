#include "data_message_functions.h"

namespace optitrack_wrapper {

::optitrack_wrapper_ros2_msgs::msg::MarkerSetData
GetMarkerSetDataMessage(sMarkerSetData *p_ms,
                        ::rclcpp::Node::SharedPtr ros_node, bool verbose) {

  ::optitrack_wrapper_ros2_msgs::msg::MarkerSetData marker_set_data;
  marker_set_data.name = p_ms->szName;
  marker_set_data.n_markers = p_ms->nMarkers;
  if (verbose) {
    RCLCPP_INFO(ros_node->get_logger(), "Marker Set name: %s", p_ms->szName);
    RCLCPP_INFO(ros_node->get_logger(), "Marker Set - number of markers : %d",
                p_ms->nMarkers);
  }

  for (int i = 0; i < p_ms->nMarkers; i++) {
    geometry_msgs::msg::Point point;
    point.x = p_ms->Markers[i][0];
    point.y = p_ms->Markers[i][1];
    point.z = p_ms->Markers[i][2];
    marker_set_data.markers.push_back(point);
    if (verbose) {
      RCLCPP_INFO(ros_node->get_logger(),
                  "Marker Set - marker %d:\t%3.2f\t%3.2f\t%3.2f", i, point.x,
                  point.y, point.z);
    }
  }
  return marker_set_data;
}

::optitrack_wrapper_ros2_msgs::msg::RigidBodyData
GetRigidBodyDataMessage(sRigidBodyData *p_rb,
                        ::rclcpp::Node::SharedPtr ros_node, bool verbose) {

  ::optitrack_wrapper_ros2_msgs::msg::RigidBodyData rigid_body_data;

  // params
  // 0x01 : bool, rigid body was successfully tracked in this frame
  bool b_tracking_valid = p_rb->params & 0x01;
  rigid_body_data.tracking_valid = b_tracking_valid;

  rigid_body_data.id = p_rb->ID;
  rigid_body_data.pose.position.x = p_rb->x;
  rigid_body_data.pose.position.y = p_rb->y;
  rigid_body_data.pose.position.z = p_rb->z;
  rigid_body_data.pose.orientation.x = p_rb->qx;
  rigid_body_data.pose.orientation.y = p_rb->qy;
  rigid_body_data.pose.orientation.z = p_rb->qz;
  rigid_body_data.pose.orientation.w = p_rb->qw;
  rigid_body_data.mean_error = p_rb->MeanError;

  if (verbose) {
    RCLCPP_INFO(ros_node->get_logger(),
                "Rigid Body [ID=%d  Error=%3.2f  Valid=%d]", p_rb->ID,
                p_rb->MeanError, b_tracking_valid);
    RCLCPP_INFO(ros_node->get_logger(), "\tx\ty\tz\tqx\tqy\tqz\tqw");
    RCLCPP_INFO(ros_node->get_logger(),
                "\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f", p_rb->x,
                p_rb->y, p_rb->z, p_rb->qx, p_rb->qy, p_rb->qz, p_rb->qw);
  }
  return rigid_body_data;
}

// get skeleton data message
::optitrack_wrapper_ros2_msgs::msg::SkeletonData
GetSkeletonDataMessage(sSkeletonData *p_sk, ::rclcpp::Node::SharedPtr ros_node,
                       bool verbose) {

  ::optitrack_wrapper_ros2_msgs::msg::SkeletonData skeleton_data;
  skeleton_data.id = p_sk->skeletonID;
  skeleton_data.n_rigid_bodies = p_sk->nRigidBodies;

  for (int i = 0; i < p_sk->nRigidBodies; i++) {
    ::optitrack_wrapper_ros2_msgs::msg::RigidBodyData rigid_body_data =
        GetRigidBodyDataMessage(&p_sk->RigidBodyData[i], ros_node, verbose);
    skeleton_data.rigid_body_data.push_back(rigid_body_data);
  }

  if (verbose) {
    RCLCPP_INFO(ros_node->get_logger(), "Skeleton [ID=%d  Bone count=%d]",
                p_sk->skeletonID, p_sk->nRigidBodies);
    for (int j = 0; j < p_sk->nRigidBodies; j++) {
      sRigidBodyData rb_data = p_sk->RigidBodyData[j];
      RCLCPP_INFO(ros_node->get_logger(),
                  "Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f",
                  rb_data.ID, rb_data.x, rb_data.y, rb_data.z, rb_data.qx,
                  rb_data.qy, rb_data.qz, rb_data.qw);
    }
  }
  return skeleton_data;
}

::optitrack_wrapper_ros2_msgs::msg ::ForcePlateData
GetForcePlateDataMessage(sForcePlateData *p_fp,
                         int analog_samples_per_mocap_frame,
                         ::rclcpp::Node::SharedPtr ros_node, bool verbose) {

  ::optitrack_wrapper_ros2_msgs::msg::ForcePlateData force_plate_data;

  force_plate_data.id = p_fp->ID;
  force_plate_data.n_channels = p_fp->nChannels;
  force_plate_data.params = p_fp->params;

  if (verbose) {
    RCLCPP_INFO(ros_node->get_logger(), "Force Plate %d", p_fp->ID);
  }

  for (int i_channel = 0; i_channel < p_fp->nChannels; i_channel++) {
    if (verbose) {
      RCLCPP_INFO(ros_node->get_logger(), "\tChannel %d:\t", i_channel);
    }
    ::optitrack_wrapper_ros2_msgs::msg::AnalogChannelData analog_channel_data =
        GetAnalogChannelDataMessage(&p_fp->ChannelData[i_channel],
                                    analog_samples_per_mocap_frame, ros_node,
                                    verbose);
    force_plate_data.channel_data.push_back(analog_channel_data);
  }
  return force_plate_data;
}

::optitrack_wrapper_ros2_msgs::msg::AnalogChannelData
GetAnalogChannelDataMessage(sAnalogChannelData *p_acd,
                            int analog_samples_per_mocap_frame,
                            ::rclcpp::Node::SharedPtr ros_node, bool verbose) {
  ::optitrack_wrapper_ros2_msgs::msg::AnalogChannelData analog_channel_data;
  analog_channel_data.n_frames = p_acd->nFrames;
  if (p_acd->nFrames == 0) {
    if (verbose) {
      RCLCPP_INFO(ros_node->get_logger(), "\tEmpty Frame");
    }
  } else if (p_acd->nFrames != analog_samples_per_mocap_frame) {
    if (verbose) {
      RCLCPP_INFO(ros_node->get_logger(),
                  "\tPartial Frame [Expected:%d   Actual:%d]",
                  analog_samples_per_mocap_frame, p_acd->nFrames);
    }
  }
  for (int i_sample = 0; i_sample < p_acd->nFrames; i_sample++) {
    analog_channel_data.values.push_back(p_acd->Values[i_sample]);
    if (verbose) {
      RCLCPP_INFO(ros_node->get_logger(), "%3.2f\t", p_acd->Values[i_sample]);
    }
  }
  return analog_channel_data;
}

::optitrack_wrapper_ros2_msgs::msg::DeviceData
GetDeviceDataMessage(sDeviceData *p_dd, int analog_samples_per_mocap_frame,
                     ::rclcpp::Node::SharedPtr ros_node, bool verbose) {

  ::optitrack_wrapper_ros2_msgs::msg::DeviceData device_data;
  device_data.id = p_dd->ID;
  device_data.n_channels = p_dd->nChannels;
  device_data.params = p_dd->params;

  if (verbose) {
    RCLCPP_INFO(ros_node->get_logger(), "Device %d", p_dd->ID);
  }

  for (int i_channel = 0; i_channel < p_dd->nChannels; i_channel++) {
    if (verbose) {
      RCLCPP_INFO(ros_node->get_logger(), "\tChannel %d:\t", i_channel);
    }
    ::optitrack_wrapper_ros2_msgs::msg::AnalogChannelData analog_channel_data =
        GetAnalogChannelDataMessage(&p_dd->ChannelData[i_channel],
                                    analog_samples_per_mocap_frame, ros_node,
                                    verbose);
    device_data.channel_data.push_back(analog_channel_data);
  }
  return device_data;
}

::optitrack_wrapper_ros2_msgs::msg::Marker
GetLabeledMarkerMessage(sMarker *p_marker, ::rclcpp::Node::SharedPtr ros_node,
                        bool verbose) {

  ::optitrack_wrapper_ros2_msgs::msg::Marker labeled_marker;

  bool b_occluded;      // marker was not visible (occluded) in this frame
  bool b_pc_solved;     // reported position provided by point cloud solve
  bool b_model_solved;  // reported position provided by model solve
  bool b_has_model;     // marker has an associated asset in the data stream
  bool b_unlabeled;     // marker is 'unlabeled', but has a point cloud ID that
                        // matches Motive PointCloud ID (In Motive 3D View)
  bool b_active_marker; // marker is an actively labeled LED marker
  bool b_established;   // marker is established
  bool b_measurement;   // measurement

  b_occluded = ((p_marker->params & 0x01) != 0);
  b_pc_solved = ((p_marker->params & 0x02) != 0);
  b_model_solved = ((p_marker->params & 0x04) != 0);
  b_has_model = ((p_marker->params & 0x08) != 0);
  b_unlabeled = ((p_marker->params & 0x10) != 0);
  b_active_marker = ((p_marker->params & 0x20) != 0);
  b_established = ((p_marker->params & 0x40) != 0);
  b_measurement = ((p_marker->params & 0x80) != 0);

  labeled_marker.occluded = b_occluded;
  labeled_marker.point_cloud_solved = b_pc_solved;
  labeled_marker.model_filled = b_model_solved;
  labeled_marker.has_model = b_has_model;
  labeled_marker.unlabeled = b_unlabeled;
  labeled_marker.active = b_active_marker;
  labeled_marker.established = b_established;
  labeled_marker.measurement = b_measurement;

  labeled_marker.id = p_marker->ID;

  labeled_marker.residual = p_marker->residual;

  // Marker ID Scheme:
  // Active Markers:
  //   ID = ActiveID, correlates to RB ActiveLabels list
  // Passive Markers:
  //   If Asset with Legacy Labels
  //      AssetID 	(Hi Word)
  //      MemberID	(Lo Word)
  //   Else
  //      PointCloud ID
  int model_id, marker_id;
  NatNet_DecodeID(p_marker->ID, &model_id, &marker_id);

  labeled_marker.model_id = model_id;
  labeled_marker.marker_id = marker_id;

  labeled_marker.size = p_marker->size;
  labeled_marker.position.x = p_marker->x;
  labeled_marker.position.y = p_marker->y;
  labeled_marker.position.z = p_marker->z;

  if (verbose) {
    char sz_marker_type[512];
    if (b_active_marker)
      strcpy(sz_marker_type, "Active");
    else if (b_unlabeled)
      strcpy(sz_marker_type, "Unlabeled");
    else
      strcpy(sz_marker_type, "Labeled");

    RCLCPP_INFO(ros_node->get_logger(),
                "%s Marker [model_id=%d, marker_id=%d] [size=%3.2f] "
                "[pos=%3.2f,%3.2f,%3.2f]",
                sz_marker_type, model_id, marker_id, p_marker->size,
                p_marker->x, p_marker->y, p_marker->z);
  }

  return labeled_marker;
}

} // namespace optitrack_wrapper
