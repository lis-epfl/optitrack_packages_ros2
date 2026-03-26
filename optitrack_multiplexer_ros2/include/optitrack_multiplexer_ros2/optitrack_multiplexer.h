#ifndef OPTITRACK_MULTIPLEXER_CLASS_H_
#define OPTITRACK_MULTIPLEXER_CLASS_H_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "optitrack_multiplexer_ros2_msgs/msg/marker_base.hpp"
#include "optitrack_multiplexer_ros2_msgs/msg/rigid_body_base.hpp"
#include "optitrack_multiplexer_ros2_msgs/msg/rigid_body_stamped.hpp"
#include "optitrack_multiplexer_ros2_msgs/msg/skeleton_stamped.hpp"
#include "optitrack_multiplexer_ros2_msgs/msg/unlabeled_markers_stamped.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/frame_of_mocap_data.hpp"
#include "optitrack_wrapper_ros2_msgs/srv/get_data_descriptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <exception>

namespace optitrack_multiplexer {

class OptitrackMultiplexer : public ::rclcpp::Node {
public:
  // constructor
  OptitrackMultiplexer();

private:
  /*---------- class methods ----------*/

  // declare ros parameters
  void DeclareRosParameters();

  // initialize member variables from ros parameters
  void InitializeRosParameters();

  // get string array from the concatenated names
  ::std::vector<::std::string> GetStringArray(::std::string str);

  // get data descriptions and wait until they arrive
  void GetDataDescriptionsSync();

  // get data descriptions and continue execution until they arrive; only way
  // not to get an error in the ProcessFrame function
  void GetDataDescriptionsAsync();

  // data descriptions service response callback
  void DataDescriptionsResponseCallback(
      ::rclcpp::Client<
          ::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions>::SharedFuture
          future);

  // generate the id vectors corresponding to the names
  void GenerateIDVectors();

  // initialize rigid body publisher
  void InitializeRigidBodyPublishers();

  // intialize skeleton publisher
  void InitializeSkeletonPublishers();

  // frame data callback
  void FrameDataCallback(
      const ::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData::SharedPtr
          frame_data);

  // get rigid body message
  ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyBase GetRigidBodyBaseMessage(
      ::optitrack_wrapper_ros2_msgs::msg::RigidBodyData rigid_body_data,
      ::std::string &rigid_body_name,
      ::std::vector<::optitrack_wrapper_ros2_msgs::msg::RigidBodyDescription>
          &rigid_body_descriptions);

  // get markers message
  ::std::vector<::optitrack_multiplexer_ros2_msgs::msg::MarkerBase>
  GetMarkerBaseMessage(
      const ::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData::SharedPtr
          frame_data,
      ::std::string &marker_set_name, int marker_set_id);

  // publish rigid body as a ros2 tf
  void PublishTf(
      const rclcpp::Time &time,
      const ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyBase &rigid_body);

  /*---------- class variables ----------*/

  // topic name
  ::std::string topic_frame_data_;

  // rigid body, skeleton and labeled marker names that we want to publish
  ::std::vector<::std::string> rigid_body_name_vec_;
  ::std::vector<::std::string> skeleton_name_vec_;
  bool publish_unlabeled_markers_;
  ::std::string topic_unlabeled_markers_;
  bool verbose_;
  bool publish_tf_;
  ::std::string world_frame_;

  // id of assets corresponding to the names
  ::std::vector<int> rigid_body_id_vec_;
  ::std::vector<int> skeleton_id_vec_;

  // boolean that indicates that the data descriptions are ready
  bool data_descriptions_ready_;

  // boolean that indicates that the id vectors are ready
  bool id_vectors_ready_;

  // data descriptions client
  ::rclcpp::Client<::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions>::
      SharedPtr data_descriptions_client_;

  // data descriptions service name
  ::std::string data_descriptions_service_name_;

  // data descriptions
  ::optitrack_wrapper_ros2_msgs::msg::DataDescriptions data_descriptions_;

  // rigid body publisher
  ::std::vector<::rclcpp::Publisher<
      ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>::SharedPtr>
      rigid_body_publisher_vec_;

  // skeleton publisher
  ::std::vector<::rclcpp::Publisher<
      ::optitrack_multiplexer_ros2_msgs::msg::SkeletonStamped>::SharedPtr>
      skeleton_publisher_vec_;

  // unlabeled markers publisher
  ::rclcpp::Publisher<
      ::optitrack_multiplexer_ros2_msgs::msg::UnlabeledMarkersStamped>::
      SharedPtr unlabeled_markers_publisher_;

  // optitrack frame data subscriber
  ::rclcpp::Subscription<::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData>::
      SharedPtr frame_data_subscriber_;

  // tf broadcaster
  ::std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace optitrack_multiplexer

#endif
