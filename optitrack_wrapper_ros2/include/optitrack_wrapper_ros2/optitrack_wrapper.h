#ifndef OPTITRACK_WRAPPER_CLASS_H_
#define OPTITRACK_WRAPPER_CLASS_H_

#include "builtin_interfaces/msg/time.hpp"
#include "data_message_functions.h"
#include "description_message_functions.h"
#include "optitrack_wrapper_ros2_msgs/msg/data_descriptions.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/frame_of_mocap_data.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/marker.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/server_description.hpp"
#include "optitrack_wrapper_ros2_msgs/srv/get_data_descriptions.hpp"
#include "optitrack_wrapper_ros2_msgs/srv/get_server_description.hpp"
#include "rclcpp/rclcpp.hpp"

#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetTypes.h>
#include <chrono>

namespace optitrack_wrapper {
class OptitrackWrapper : public ::rclcpp::Node {
public:
  // constructor
  OptitrackWrapper();

  // process frame data: can be a private friend function
  // that can access the private variables and functions of this class instead
  // of having a proxy function; for now we prefer a proxy function because of
  // NATNET_CALLCONV
  void ProcessFrame(sFrameOfMocapData *data);

  void SetupNatNet();

private:
  /*---------- class methods ----------*/


  // declare ros connection parameters
  void DeclareRosParameters();

  // get ros parameters from file
  void InitializeRosParameters();

  // initialize NatNetClient
  void InitializeClient();

  // initialize client parameters
  void InitializeClientParams();

  // connect to optitrack
  void ConnectOptitrack();

  // get data descriptions
  void GetDataDescriptions();

  // print data descriptions
  void PrintDataDescriptions();

  // create data descritions message
  void CreateDataDescriptionsMessage();

  // clear data descrition
  void ClearDataDescriptions();

  // create get sesrver description service
  void GetServerDescriptionService(
      const ::std::shared_ptr<
          ::optitrack_wrapper_ros2_msgs::srv::GetServerDescription::Request>
          request,
      ::std::shared_ptr<
          ::optitrack_wrapper_ros2_msgs::srv::GetServerDescription::Response>
          response);

  // create get server description service
  void GetDataDescriptionsService(
      const ::std::shared_ptr<
          ::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions::Request>
          request,
      ::std::shared_ptr<
          ::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions::Response>
          response);

  /*---------- class variables ----------*/

  // NatNet client
  NatNetClient *nat_net_client_;

  // client parameters
  sNatNetClientConnectParams nat_net_client_params_;

  // NatNet server description
  sServerDescription server_description_;

  // data descriptions message
  ::optitrack_wrapper_ros2_msgs::msg::ServerDescription server_description_msg_;

  // data descriptions
  sDataDescriptions *data_descriptions_{nullptr};

  // data descriptions message
  ::optitrack_wrapper_ros2_msgs::msg::DataDescriptions data_descriptions_msg_;

  // frame data
  sFrameOfMocapData frame_data_;

  // frame data msg
  ::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData frame_data_msg_;

  // analog samples per frame
  int analog_samples_per_mocap_frame_{0};

  // publisher that publishes a frame of mocap data that is received from
  // NatNet SDK periodically
  ::rclcpp::Publisher<::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData>::
      SharedPtr mocap_data_publisher_;

  // server description service
  ::rclcpp::Service<::optitrack_wrapper_ros2_msgs::srv::GetServerDescription>::
      SharedPtr server_description_service_;

  // server description service
  ::rclcpp::Service<::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions>::
      SharedPtr data_descriptions_service_;

  // connection parameters
  ::std::string connection_type_;
  ::std::string server_address_;
  ::std::string local_address_;
  ::std::string multicast_address_;
  uint16_t server_command_port_;
  uint16_t server_data_port_;

  // verbose parameters
  bool verbose_data_description_;
  bool verbose_frame_;

  // frame data publisher topic name
  ::std::string topic_frame_data_;
};

// proxy function to handle data received from motive
void NATNET_CALLCONV ProcessFrameCallback(sFrameOfMocapData *data,
                                          void *p_user_data);

} // namespace optitrack_wrapper

#endif
