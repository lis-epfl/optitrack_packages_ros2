#include "optitrack_multiplexer.h"

namespace optitrack_multiplexer {

OptitrackMultiplexer::OptitrackMultiplexer()
    : ::rclcpp::Node("optitrack_multiplexer") {

  // declare connection parameters
  DeclareRosParameters();

  // get parameters from file
  InitializeRosParameters();

  // initialize data descriptions client
  data_descriptions_client_ =
      create_client<::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions>(
          data_descriptions_service_name_);

  // get data descriptions
  GetDataDescriptionsSync();

  // generate the id vectors corresponding to the names
  GenerateIDVectors();

  // initialize rigid body publisher
  InitializeRigidBodyPublishers();

  // initialize skeleton publisher
  InitializeSkeletonPublishers();

  // initialize unlabeled markers publisher
  unlabeled_markers_publisher_ = create_publisher<
      ::optitrack_multiplexer_ros2_msgs::msg::UnlabeledMarkersStamped>(
      "~/" + topic_unlabeled_markers_, 10);

  // initialize frame data subscriber
  frame_data_subscriber_ =
      create_subscription<::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData>(
          topic_frame_data_, 10,
          std::bind(&OptitrackMultiplexer::FrameDataCallback, this,
                    ::std::placeholders::_1));
}

void OptitrackMultiplexer::FrameDataCallback(
    const ::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData::SharedPtr
        frame_data) {
  // check if the list of assests has changed; if yes get new data descriptions
  if (frame_data->model_list_changed) {
    // get data descriptions
    GetDataDescriptionsAsync();
    RCLCPP_INFO(get_logger(), "Model list changed.");
  }

  // only process the frame if we have the new data descriptions
  if (data_descriptions_ready_) {
    /* RCLCPP_INFO(get_logger(), "Here"); */
    if (!id_vectors_ready_) {
      GenerateIDVectors();
    }

    // check ros2 transit time
    ::rclcpp::Time time_now = now();
    double ros2_transit_dt =
        (time_now - frame_data->time_publishing).nanoseconds();
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "Transit delay ros2: %f ms",
                  1e-6 * ros2_transit_dt);
    }

    // start new timer for current data handling
    auto processing_time_start = ::std::chrono::high_resolution_clock::now();

    /*---------- rigid bodies -----------*/

    // extract rigid body information and create the message
    ::std::vector<::optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>
        rigid_body_stamped_msgs;

    // vector to save the ids that have been found and use their index to select
    // the correct publisher later on
    ::std::vector<int> rigid_body_publisher_id;

    for (int i = 0; i < frame_data->rigid_bodies.size(); i++) {
      // check if id is found in the id vector
      auto rigid_body_vec_idx =
          ::std::find(rigid_body_id_vec_.begin(), rigid_body_id_vec_.end(),
                      frame_data->rigid_bodies[i].id);
      if (rigid_body_vec_idx != rigid_body_id_vec_.end()) {
        // if the id is found in the id vector
        // save the name id to use with the publisher id vector
        int name_idx = rigid_body_vec_idx - rigid_body_id_vec_.begin();
        rigid_body_publisher_id.push_back(name_idx);

        //  create the rigid body message
        ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped rigid_body_msg;
        ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyBase rigid_body_base =
            GetRigidBodyBaseMessage(frame_data->rigid_bodies[i],
                                    rigid_body_name_vec_[name_idx],
                                    data_descriptions_.rigid_body_descriptions);
        rigid_body_msg.rigid_body = rigid_body_base;

        // add the markers information
        ::std::vector<::optitrack_multiplexer_ros2_msgs::msg::MarkerBase>
            markers = GetMarkerBaseMessage(frame_data, rigid_body_base.name,
                                           rigid_body_base.id);
        rigid_body_msg.markers = markers;

        // add the msg to the message vectors
        rigid_body_stamped_msgs.push_back(rigid_body_msg);
      }
    }

    /*---------- skeletons -----------*/

    // extract skeleton information and create the message
    ::std::vector<::optitrack_multiplexer_ros2_msgs::msg::SkeletonStamped>
        skeleton_stamped_msgs;

    // vector to save the ids that have been found and use their index to select
    // the correct publisher later on
    ::std::vector<int> skeleton_publisher_id;

    for (int i = 0; i < frame_data->skeletons.size(); i++) {
      // find the index in the id vector that correxpond to the id of the
      // skeleton
      auto skeleton_vec_idx =
          ::std::find(skeleton_id_vec_.begin(), skeleton_id_vec_.end(),
                      frame_data->skeletons[i].id);

      if (skeleton_vec_idx != skeleton_id_vec_.end()) {
        // if the id is found in the id vector
        // save the name id to use with the publisher id vector
        int name_idx = skeleton_vec_idx - skeleton_id_vec_.begin();
        skeleton_publisher_id.push_back(name_idx);

        //  create the rigid body message
        ::optitrack_multiplexer_ros2_msgs::msg::SkeletonStamped skeleton_msg;

        // set the skeleton id
        skeleton_msg.id = frame_data->skeletons[i].id;

        // find rigid body descriptions
        ::std::vector<::optitrack_wrapper_ros2_msgs::msg::RigidBodyDescription>
            rigid_body_descriptions;
        // first find skeleton description
        for (int j = 0; j < data_descriptions_.n_skeletons; j++) {
          if (data_descriptions_.skeleton_descriptions[j].name ==
              skeleton_name_vec_[name_idx]) {
            // then set rigid body descriptions
            rigid_body_descriptions =
                data_descriptions_.skeleton_descriptions[j]
                    .rigid_body_descriptions;
          }
        }

        // get rigid body base using rigid body data and description
        for (int j = 0; j < frame_data->skeletons[i].n_rigid_bodies; j++) {
          ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyBase
              rigid_body_base = GetRigidBodyBaseMessage(
                  frame_data->skeletons[i].rigid_body_data[j],
                  rigid_body_descriptions[j].name, rigid_body_descriptions);
          skeleton_msg.rigid_bodies.push_back(rigid_body_base);
        }

        // add the markers information
        ::std::vector<::optitrack_multiplexer_ros2_msgs::msg::MarkerBase>
            markers = GetMarkerBaseMessage(
                frame_data, skeleton_name_vec_[name_idx], skeleton_msg.id);
        skeleton_msg.markers = markers;

        // add the msg to the message vectors
        skeleton_stamped_msgs.push_back(skeleton_msg);
      }
    }

    /*---------- unlabeled_markers -----------*/
    ::optitrack_multiplexer_ros2_msgs::msg::UnlabeledMarkersStamped
        unlabeled_markers_stamped_msg;

    for (int i = 0; i < frame_data->n_labeled_markers; i++) {
      if (frame_data->labeled_markers[i].model_id == 0 ||
          frame_data->labeled_markers[i].model_id == 1) {
        unlabeled_markers_stamped_msg.positions.push_back(
            frame_data->labeled_markers[i].position);
        unlabeled_markers_stamped_msg.id.push_back(
            frame_data->labeled_markers[i].id);
      }
    }

    /*---------- publish all info -----------*/

    // end new timer for current data handling
    auto processing_time_end = ::std::chrono::high_resolution_clock::now();
    auto processing_duration_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            processing_time_end - processing_time_start);
    double total_pipeline_latency =
        frame_data->latency_camera_mid_exposure_to_data_received_ms +
        frame_data->latency_camera_data_received_to_transmission_ms +
        frame_data->latency_transmission_to_client_ms + 1e-6 * ros2_transit_dt +
        1e-6 * processing_duration_ns.count();

    if (verbose_) {
      RCLCPP_INFO(get_logger(), "Data handling latency: %f ms",
                  1e-6 * processing_duration_ns.count());
      RCLCPP_INFO(get_logger(), "Total pipeline latency: %f ms",
                  total_pipeline_latency);
    }

    // publish rigid bodies
    for (int i = 0; i < rigid_body_publisher_id.size(); i++) {

      ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped
          rigid_body_stamped_msg = rigid_body_stamped_msgs[i];

      rigid_body_stamped_msg.stamp = now();
      rigid_body_stamped_msg.frame = frame_data->frame;
      rigid_body_stamped_msg.latency_ms = total_pipeline_latency;

      rigid_body_publisher_vec_[rigid_body_publisher_id[i]]->publish(
          rigid_body_stamped_msg);
    }

    // publish skeletons
    for (int i = 0; i < skeleton_publisher_id.size(); i++) {

      ::optitrack_multiplexer_ros2_msgs::msg::SkeletonStamped
          skeleton_stamped_msg = skeleton_stamped_msgs[i];
      skeleton_stamped_msg.stamp = now();
      skeleton_stamped_msg.frame = frame_data->frame;
      skeleton_stamped_msg.latency_ms = total_pipeline_latency;

      skeleton_publisher_vec_[skeleton_publisher_id[i]]->publish(
          skeleton_stamped_msg);
    }

    // publish unlabeled markers
    if (unlabeled_markers_stamped_msg.positions.size() != 0) {
      unlabeled_markers_stamped_msg.stamp = now();
      unlabeled_markers_stamped_msg.frame = frame_data->frame;
      unlabeled_markers_stamped_msg.latency_ms = total_pipeline_latency;
      unlabeled_markers_publisher_->publish(unlabeled_markers_stamped_msg);
    }
  }
}

void OptitrackMultiplexer::DataDescriptionsResponseCallback(
    ::rclcpp::Client<::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions>::
        SharedFuture future) {

  auto status = future.wait_for(::std::chrono::seconds(1));
  if (status == ::std::future_status::ready) {
    // store the data descriptions
    data_descriptions_ = future.get()->data_descriptions;
    data_descriptions_ready_ = true;
  }
}

::optitrack_multiplexer_ros2_msgs::msg::RigidBodyBase
OptitrackMultiplexer::GetRigidBodyBaseMessage(
    ::optitrack_wrapper_ros2_msgs::msg::RigidBodyData rigid_body_data,
    ::std::string &rigid_body_name,
    ::std::vector<::optitrack_wrapper_ros2_msgs::msg::RigidBodyDescription>
        &rigid_body_descriptions) {

  // get rigid body data
  ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyBase rigid_body;
  rigid_body.pose = rigid_body_data.pose;
  rigid_body.id = rigid_body_data.id;
  rigid_body.mean_error = rigid_body_data.mean_error;
  rigid_body.tracking_valid = rigid_body_data.tracking_valid;

  // get parent id, name (and potentially pattern from rigid body description)
  for (int i = 0; i < rigid_body_descriptions.size(); i++) {
    if (rigid_body_descriptions[i].name == rigid_body_name) {
      rigid_body.parent_id = rigid_body_descriptions[i].parent_id;
      rigid_body.name = rigid_body_descriptions[i].name;
    }
  }
  return rigid_body;
}

::std::vector<::optitrack_multiplexer_ros2_msgs::msg::MarkerBase>
OptitrackMultiplexer::GetMarkerBaseMessage(
    const ::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData::SharedPtr
        frame_data,
    ::std::string &marker_set_name, int marker_set_id) {

  // define markers to return
  ::std::vector<::optitrack_multiplexer_ros2_msgs::msg::MarkerBase> markers;
  ::optitrack_multiplexer_ros2_msgs::msg::MarkerBase marker;

  // get marker names from description cause they don't exist in rigid
  // body/skeleton description
  for (int i = 0; i < data_descriptions_.n_marker_sets; i++) {
    if (data_descriptions_.marker_set_descriptions[i].name == marker_set_name) {
      for (int j = 0;
           j < data_descriptions_.marker_set_descriptions[i].n_markers; j++) {
        marker.name =
            data_descriptions_.marker_set_descriptions[i].marker_names[j];
        markers.push_back(marker);
      }
      break;
    }
  }

  // get marker positions from the marker set data
  for (int i = 0; i < frame_data->n_marker_sets; i++) {
    if (frame_data->marker_set_data[i].name == marker_set_name) {
      for (int j = 0; j < frame_data->marker_set_data[i].n_markers; j++) {
        markers[j].position = frame_data->marker_set_data[i].markers[j];
      }
      break;
    }
  }

  // get whether the marker is model filled or visible; need to check if it
  // belongs to a model or not
  for (int i = 0; i < frame_data->n_labeled_markers; i++) {
    if (frame_data->labeled_markers[i].model_id == marker_set_id &&
        frame_data->labeled_markers[i].has_model) {
      markers[frame_data->labeled_markers[i].marker_id - 1].visible = true;
    }
  }

  return markers;
}

void OptitrackMultiplexer::InitializeRigidBodyPublishers() {
  for (int i = 0; i < rigid_body_name_vec_.size(); i++) {
    ::rclcpp::Publisher<
        ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>::SharedPtr
        rigid_body_publisher = create_publisher<
            ::optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>(
            "~/rigid_body/" + rigid_body_name_vec_[i], 10);
    rigid_body_publisher_vec_.push_back(rigid_body_publisher);
  }
}

void OptitrackMultiplexer::InitializeSkeletonPublishers() {
  for (int i = 0; i < skeleton_name_vec_.size(); i++) {
    ::rclcpp::Publisher<
        ::optitrack_multiplexer_ros2_msgs::msg::SkeletonStamped>::SharedPtr
        skeleton_publisher = create_publisher<
            ::optitrack_multiplexer_ros2_msgs::msg::SkeletonStamped>(
            "~/skeleton/" + skeleton_name_vec_[i], 10);
    skeleton_publisher_vec_.push_back(skeleton_publisher);
  }
}

void OptitrackMultiplexer::GenerateIDVectors() {

  // run through the array of rigid bodies
  auto rigid_body_descriptions = data_descriptions_.rigid_body_descriptions;
  rigid_body_id_vec_.clear();
  rigid_body_id_vec_.resize(rigid_body_name_vec_.size(), 0);
  for (int i = 0; i < data_descriptions_.n_rigid_bodies; i++) {
    for (int j = 0; j < rigid_body_name_vec_.size(); j++) {
      if (rigid_body_descriptions[i].name == rigid_body_name_vec_[j]) {
        rigid_body_id_vec_[j] = rigid_body_descriptions[i].id;
      }
    }
  }

  // run through the array of skeleton
  auto skeleton_descriptions = data_descriptions_.skeleton_descriptions;
  skeleton_id_vec_.clear();
  skeleton_id_vec_.resize(skeleton_name_vec_.size(), 0);
  for (int i = 0; i < data_descriptions_.n_skeletons; i++) {
    for (int j = 0; j < skeleton_name_vec_.size(); j++) {
      if (skeleton_descriptions[i].name == skeleton_name_vec_[j]) {
        skeleton_id_vec_[j] = skeleton_descriptions[i].id;
      }
    }
  }

  // indicate that the id vectors are ready
  id_vectors_ready_ = true;
}

void OptitrackMultiplexer::GetDataDescriptionsAsync() {
  // set the ready variables to false
  data_descriptions_ready_ = false;
  id_vectors_ready_ = false;

  auto request = ::std::make_shared<
      ::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions::Request>();

  while (
      !data_descriptions_client_->wait_for_service(::std::chrono::seconds(1))) {
    if (!::rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Data descriptions service not available, waiting again...");
  }
  auto result = data_descriptions_client_->async_send_request(
      request,
      ::std::bind(&OptitrackMultiplexer::DataDescriptionsResponseCallback, this,
                  ::std::placeholders::_1));
}

void OptitrackMultiplexer::GetDataDescriptionsSync() {
  auto request = ::std::make_shared<
      ::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions::Request>();

  while (
      !data_descriptions_client_->wait_for_service(::std::chrono::seconds(1))) {
    if (!::rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Data descriptions service not available, waiting again...");
  }

  auto result = data_descriptions_client_->async_send_request(request);
  ::rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
  data_descriptions_ = result.get()->data_descriptions;
  data_descriptions_ready_ = true;
}

void OptitrackMultiplexer::DeclareRosParameters() {
  declare_parameter<::std::string>("topic_frame_data",
                                   "/optitrack_wrapper_ros/frame_data");
  declare_parameter<::std::string>("rigid_body_names", "");
  declare_parameter<::std::string>("skeleton_names", "");
  declare_parameter<bool>("publish_unlabeled_markers", false);
  declare_parameter<::std::string>("data_descriptions_service_name", "");
  declare_parameter<::std::string>("topic_unlabeled_markers",
                                   "unlabeled_markers");
  declare_parameter<bool>("verbose", true);
}

void OptitrackMultiplexer::InitializeRosParameters() {
  ::std::string rigid_body_names;
  ::std::string skeleton_names;

  get_parameter<::std::string>("topic_frame_data", topic_frame_data_);
  get_parameter<::std::string>("rigid_body_names", rigid_body_names);
  get_parameter<::std::string>("skeleton_names", skeleton_names);
  get_parameter<bool>("publish_unlabeled_markers", publish_unlabeled_markers_);
  get_parameter<::std::string>("data_descriptions_service_name",
                               data_descriptions_service_name_);
  get_parameter<::std::string>("topic_unlabeled_markers",
                               topic_unlabeled_markers_);
  get_parameter<bool>("verbose", verbose_);

  RCLCPP_INFO(get_logger(), "Rigid body names: %s", rigid_body_names.c_str());

  // generate name vectors: from one string delimted by a comma ',' to an array
  rigid_body_name_vec_ = GetStringArray(rigid_body_names);
  skeleton_name_vec_ = GetStringArray(skeleton_names);
}

::std::vector<::std::string>
OptitrackMultiplexer::GetStringArray(::std::string str) {
  ::std::vector<::std::string> str_vec;

  // remove all empty spaces in the string
  str.erase(remove_if(str.begin(), str.end(), ::isspace), str.end());

  // use strtok to split the string
  char *delimiter = (char *)",";
  char *token = strtok(const_cast<char *>(str.c_str()), delimiter);
  while (token != nullptr) {
    str_vec.push_back(::std::string(token));
    token = strtok(nullptr, delimiter);
  }

  return str_vec;
}

} // namespace optitrack_multiplexer
