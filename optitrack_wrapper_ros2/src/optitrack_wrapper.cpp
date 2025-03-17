#include "optitrack_wrapper.h"

namespace optitrack_wrapper {

OptitrackWrapper::OptitrackWrapper() : ::rclcpp::Node("optitrack_wrapper") {

  // declare connection parameters
  DeclareRosParameters();

  // get parameters from file
  InitializeRosParameters();

  // create get server description service
  server_description_service_ =
      create_service<::optitrack_wrapper_ros2_msgs::srv::GetServerDescription>(
          "~/get_server_description",
          ::std::bind(&OptitrackWrapper::GetServerDescriptionService, this,
                      ::std::placeholders::_1, ::std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Ready to receive server description requests.");

  // create get data descriptions service
  data_descriptions_service_ =
      create_service<::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions>(
          "~/get_data_descriptions",
          ::std::bind(&OptitrackWrapper::GetDataDescriptionsService, this,
                      ::std::placeholders::_1, ::std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Ready to receive data descriptions requests.");

  // initialize mocap data frame publisher
  // add "~/" which adds the node name to the topic name i.e. the node namespace
  mocap_data_publisher_ =
      create_publisher<::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData>(
          "~/" + topic_frame_data_, 0);

  // initialize client
  InitializeClient();

  // initialize client params
  InitializeClientParams();

  // connect optitrack
  ConnectOptitrack();

  // get data descriptions
  GetDataDescriptions();

  // create data description message
  CreateDataDescriptionsMessage();

  // initialize frame data msg
  frame_data_msg_ = ::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData();
}

void NATNET_CALLCONV ProcessFrameCallback(sFrameOfMocapData *data,
                                          void *p_user_data) {
  static_cast<OptitrackWrapper *>(p_user_data)->ProcessFrame(data);
}

void OptitrackWrapper::GetServerDescriptionService(
    const ::std::shared_ptr<
        ::optitrack_wrapper_ros2_msgs::srv::GetServerDescription::Request>
        request,
    ::std::shared_ptr<
        ::optitrack_wrapper_ros2_msgs::srv::GetServerDescription::Response>
        response) {

  response->server_description = server_description_msg_;
}

void OptitrackWrapper::GetDataDescriptionsService(
    const ::std::shared_ptr<
        ::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions::Request>
        request,
    ::std::shared_ptr<
        ::optitrack_wrapper_ros2_msgs::srv::GetDataDescriptions::Response>
        response) {

  GetDataDescriptions();
  CreateDataDescriptionsMessage();
  response->data_descriptions = data_descriptions_msg_;
}

void OptitrackWrapper::ProcessFrame(sFrameOfMocapData *data) {
  const int MAX_TRIES = 10; 
  int tryno = 0; 
  auto shared_ptr = weak_from_this().lock(); 

  while(!shared_ptr && tryno < MAX_TRIES) {
    shared_ptr = weak_from_this().lock();
    tryno++;
    RCLCPP_INFO(get_logger(), "Could not get shared pointer for node, try %d/%d", tryno + 1, MAX_TRIES);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  if(!shared_ptr) {
    // we could not get the pointer, fail. 
    RCLCPP_ERROR(get_logger(), "Could not get the shared pointer at all, exiting");
    rclcpp::shutdown();
    exit(1);
  }


  // Start measuring time
  auto processing_time_begin = ::std::chrono::high_resolution_clock::now();

  // save data in a member variable
  frame_data_ = *data;

  // define message for frame of mocap data
  ::optitrack_wrapper_ros2_msgs::msg::FrameOfMocapData frame_data_msg;

  // Software latency here is defined as the span of time between:
  //   a) The reception of a complete group of 2D frames from the camera system
  //   (CameraDataReceivedTimestamp)
  // and
  //   b) The time immediately prior to the NatNet frame being transmitted over
  //   the network (TransmitTimestamp)
  //
  // This figure may appear slightly higher than the "software latency" reported
  // in the Motive user interface, because it additionally includes the time
  // spent preparing to stream the data via NatNet.
  const uint64_t software_latency_host_ticks =
      data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
  const double software_latency_ms =
      (software_latency_host_ticks * 1000) /
      static_cast<double>(server_description_.HighResClockFrequency);

  frame_data_msg.latency_camera_data_received_to_transmission_ms =
      software_latency_ms;

  // Transit latency is defined as the span of time between Motive transmitting
  // the frame of data, and its reception by the client (now). The
  // SecondsSinceHostTimestamp method relies on NatNetClient's internal clock
  // synchronization with the server using Cristian's algorithm.
  const double transit_latency_ms =
      nat_net_client_->SecondsSinceHostTimestamp(data->TransmitTimestamp) *
      1000.0;

  frame_data_msg.latency_transmission_to_client_ms = transit_latency_ms;

  frame_data_msg.frame = data->iFrame;
  frame_data_msg.timestamp = data->fTimestamp;
  frame_data_msg.camera_mid_exposure_timestamp =
      data->CameraMidExposureTimestamp;
  frame_data_msg.camera_data_received_timestamp =
      data->CameraDataReceivedTimestamp;
  frame_data_msg.transmit_timestamp = data->TransmitTimestamp;

  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(), "\n");
    RCLCPP_INFO(get_logger(), "FrameID : %d", data->iFrame);
    RCLCPP_INFO(get_logger(), "Timestamp : %3.2lf", data->fTimestamp);
    RCLCPP_INFO(get_logger(), "Software latency : %.2lf milliseconds",
                software_latency_ms);
  }

  // Only recent versions of the Motive software in combination with ethernet
  // camera systems support system latency measurement. If it's unavailable (for
  // example, with USB camera systems, or during playback), this field will be
  // zero.
  const bool b_system_latency_available = data->CameraMidExposureTimestamp != 0;

  if (b_system_latency_available) {
    // System latency here is defined as the span of time between:
    //   a) The midpoint of the camera exposure window, and therefore the
    //   average age of the photons (CameraMidExposureTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted
    //   over the network (TransmitTimestamp)
    const uint64_t system_latency_host_ticks =
        data->TransmitTimestamp - data->CameraMidExposureTimestamp;
    const double system_latency_ms =
        (system_latency_host_ticks * 1000) /
        static_cast<double>(server_description_.HighResClockFrequency);

    frame_data_msg.latency_camera_mid_exposure_to_data_received_ms =
        system_latency_ms - software_latency_ms;

    // Client latency is defined as the sum of system latency and the transit
    // time taken to relay the data to the NatNet client. This is the
    // all-inclusive measurement (photons to client processing).
    const double client_latency_ms = nat_net_client_->SecondsSinceHostTimestamp(
                                         data->CameraMidExposureTimestamp) *
                                     1000.0;

    // You could equivalently do the following (not accounting for time elapsed
    // since we calculated transit latency above):
    // const double client_latency_ms = system_latency_ms +
    // transit_latency_ms;

    if (verbose_frame_) {
      RCLCPP_INFO(get_logger(), "System latency : %.2lf milliseconds",
                  system_latency_ms);
      RCLCPP_INFO(
          get_logger(),
          "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)",
          client_latency_ms, transit_latency_ms);
    }
  } else {
    if (verbose_frame_) {
      RCLCPP_INFO(get_logger(), "Transit latency : %.2lf milliseconds",
                  transit_latency_ms);
    }
  }

  // FrameOfMocapData params
  bool b_is_recording = ((data->params & 0x01) != 0);
  bool b_tracked_models_changed = ((data->params & 0x02) != 0);
  frame_data_msg.recording = b_is_recording;
  frame_data_msg.model_list_changed = b_tracked_models_changed;

  if (verbose_frame_) {
    if (b_is_recording)
      RCLCPP_INFO(get_logger(), "RECORDING");
    if (b_tracked_models_changed)
      RCLCPP_INFO(get_logger(), "Models Changed.");
  }

  // timecode - for systems with an eSync and SMPTE timecode generator - decode
  // to values
  int hour, minute, second, frame, subframe;
  NatNet_DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute,
                        &second, &frame, &subframe);

  // decode to friendly string
  char sz_timecode[128] = "";
  NatNet_TimecodeStringify(data->Timecode, data->TimecodeSubframe, sz_timecode,
                           128);

  frame_data_msg.timecode = sz_timecode;

  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(), "Timecode : %s", sz_timecode);
  }

  // undefined markers
  frame_data_msg.n_other_markers = data->nOtherMarkers;

  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(), "Undefined Markers number: %d",
                data->nOtherMarkers);
  }

  for (int i = 0; i < data->nOtherMarkers; i++) {
    ::optitrack_wrapper_ros2_msgs::msg::Point point;
    point.x = data->OtherMarkers[i][0];
    point.y = data->OtherMarkers[i][1];
    point.z = data->OtherMarkers[i][2];
    frame_data_msg.other_markers.push_back(point);
    if (verbose_frame_) {
      RCLCPP_INFO(get_logger(),
                  "Undefined Markers position:\t%3.2f\t%3.2f\t%3.2f", point.x,
                  point.y, point.z);
    }
  }

  // Marker Sets
  frame_data_msg.n_marker_sets = data->nMarkerSets;

  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(), "Marker Sets [Count=%d]", data->nMarkerSets);
  }

  for (int i = 0; i < data->nMarkerSets; i++) {
    ::optitrack_wrapper_ros2_msgs::msg::MarkerSetData marker_set_data =
        ::optitrack_wrapper::GetMarkerSetDataMessage(
            &data->MocapData[i], shared_ptr, verbose_frame_);

    frame_data_msg.marker_set_data.push_back(marker_set_data);
  }

  // check if number of marker sets has changed (added a new body) or name
  // changed; both of these conditions are not included in the
  // b_tracked_models_changed bit
  if (frame_data_msg.n_marker_sets != frame_data_msg_.n_marker_sets) {
    frame_data_msg.model_list_changed = true;
  } else {
    // check if names changed
    for (int i = 0; i < frame_data_msg.n_marker_sets; i++) {
      if (frame_data_msg.marker_set_data[i].name !=
          frame_data_msg_.marker_set_data[i].name) {
        frame_data_msg.model_list_changed = true;
      }
    }
  }

  // Rigid Bodies
  frame_data_msg.n_rigid_bodies = data->nRigidBodies;

  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(), "Rigid Bodies [Count=%d]", data->nRigidBodies);
  }
  for (int i = 0; i < data->nRigidBodies; i++) {
    ::optitrack_wrapper_ros2_msgs::msg::RigidBodyData rigid_body_data =
        ::optitrack_wrapper::GetRigidBodyDataMessage(
            &data->RigidBodies[i], shared_ptr, verbose_frame_);

    frame_data_msg.rigid_bodies.push_back(rigid_body_data);
  }

  // Skeletons
  frame_data_msg.n_skeletons = data->nSkeletons;

  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(), "Skeletons [Count=%d]", data->nSkeletons);
  }
  for (int i = 0; i < data->nSkeletons; i++) {
    ::optitrack_wrapper_ros2_msgs::msg::SkeletonData skeleton_data =
        ::optitrack_wrapper::GetSkeletonDataMessage(
            &data->Skeletons[i], shared_ptr, verbose_frame_);

    frame_data_msg.skeletons.push_back(skeleton_data);
  }

  // labeled markers - this includes all markers (Active,
  // Passive, and 'unlabeled' (markers with no asset but a
  // PointCloud ID)
  frame_data_msg.n_labeled_markers = data->nLabeledMarkers;

  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(), "Markers [Count=%d]", data->nLabeledMarkers);
  }

  for (int i = 0; i < data->nLabeledMarkers; i++) {
    ::optitrack_wrapper_ros2_msgs::msg::Marker labeled_marker =
        ::optitrack_wrapper::GetLabeledMarkerMessage(
            &data->LabeledMarkers[i], shared_ptr, verbose_frame_);
    frame_data_msg.labeled_markers.push_back(labeled_marker);
  }

  // force plates
  frame_data_msg.n_force_plates = data->nForcePlates;
  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(), "Force Plate [Count=%d]", data->nForcePlates);
  }

  for (int i = 0; i < data->nForcePlates; i++) {
    ::optitrack_wrapper_ros2_msgs::msg::ForcePlateData force_plate_data =
        ::optitrack_wrapper::GetForcePlateDataMessage(
            &data->ForcePlates[i], analog_samples_per_mocap_frame_,
            shared_ptr, verbose_frame_);

    frame_data_msg.force_plates.push_back(force_plate_data);
  }

  // devices
  frame_data_msg.n_devices = data->nDevices;
  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(), "Device [Count=%d]", data->nDevices);
  }

  for (int i = 0; i < data->nDevices; i++) {
    ::optitrack_wrapper_ros2_msgs::msg::DeviceData device_data =
        ::optitrack_wrapper::GetDeviceDataMessage(
            &data->Devices[i], analog_samples_per_mocap_frame_,
            shared_ptr, verbose_frame_);

    frame_data_msg.devices.push_back(device_data);
  }

  // set the time before publishing
  frame_data_msg.time_publishing = now();

  // compute data handling duration
  auto processing_time_end = ::std::chrono::high_resolution_clock::now();
  auto processing_duration_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          processing_time_end - processing_time_begin);
  frame_data_msg.latency_data_handling_ms =
      1e-6 * processing_duration_ns.count();

  if (verbose_frame_) {
    RCLCPP_INFO(get_logger(),
                "Data handling latency (not real one because "
                "displaying adds "
                "to the handling latency: use ros2 topic echo "
                "to see the "
                "actual latency) %.3lf milliseconds",
                frame_data_msg.latency_data_handling_ms);
  }

  // publish the frame
  mocap_data_publisher_->publish(frame_data_msg);

  // save the frame message
  frame_data_msg_ = frame_data_msg;
}

void OptitrackWrapper::ClearDataDescriptions() {
  data_descriptions_msg_.marker_set_descriptions.clear();
  data_descriptions_msg_.rigid_body_descriptions.clear();
  data_descriptions_msg_.skeleton_descriptions.clear();
  data_descriptions_msg_.force_plate_descriptions.clear();
  data_descriptions_msg_.device_descriptions.clear();
  data_descriptions_msg_.camera_descriptions.clear();

  data_descriptions_msg_.n_marker_sets = 0;
  data_descriptions_msg_.n_rigid_bodies = 0;
  data_descriptions_msg_.n_skeletons = 0;
  data_descriptions_msg_.n_force_plates = 0;
  data_descriptions_msg_.n_devices = 0;
  data_descriptions_msg_.n_cameras = 0;
}

void OptitrackWrapper::CreateDataDescriptionsMessage() {
  // clear the msg
  ClearDataDescriptions();

  // create the msg
  for (int i = 0; i < data_descriptions_->nDataDescriptions; i++) {
    if (data_descriptions_->arrDataDescriptions[i].type ==
        Descriptor_MarkerSet) {
      // increase number of marker sets
      data_descriptions_msg_.n_marker_sets++;

      // MarkerSet
      sMarkerSetDescription *p_ms =
          data_descriptions_->arrDataDescriptions[i].Data.MarkerSetDescription;
      ::optitrack_wrapper_ros2_msgs::msg::MarkerSetDescription
          marker_set_description =
              ::optitrack_wrapper::GetMarkerSetDescriptionMessage(p_ms);

      // marker set description add to msg
      data_descriptions_msg_.marker_set_descriptions.push_back(
          marker_set_description);

    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_RigidBody) {
      // increase number of marker sets
      data_descriptions_msg_.n_rigid_bodies++;

      // RigidBody
      sRigidBodyDescription *p_rb =
          data_descriptions_->arrDataDescriptions[i].Data.RigidBodyDescription;
      ::optitrack_wrapper_ros2_msgs::msg::RigidBodyDescription
          rigid_body_description =
              ::optitrack_wrapper::GetRigidBodyDescriptionMessage(p_rb);

      // rigid body description add to msg
      data_descriptions_msg_.rigid_body_descriptions.push_back(
          rigid_body_description);

    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_Skeleton) {
      // increase number of skeletons
      data_descriptions_msg_.n_skeletons++;

      // Skeleton
      sSkeletonDescription *p_sk =
          data_descriptions_->arrDataDescriptions[i].Data.SkeletonDescription;
      ::optitrack_wrapper_ros2_msgs::msg::SkeletonDescription
          skeleton_description =
              ::optitrack_wrapper::GetSkeletonDescriptionMessage(p_sk);

      // skeleton description add to msg
      data_descriptions_msg_.skeleton_descriptions.push_back(
          skeleton_description);

    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_ForcePlate) {
      // increase number of foce plates
      data_descriptions_msg_.n_force_plates++;

      // Force Plate
      sForcePlateDescription *p_fp =
          data_descriptions_->arrDataDescriptions[i].Data.ForcePlateDescription;
      ::optitrack_wrapper_ros2_msgs::msg::ForcePlateDescription
          force_plate_description =
              ::optitrack_wrapper::GetForcePlateDescriptionMessage(p_fp);

      // force plate description add to msg
      data_descriptions_msg_.force_plate_descriptions.push_back(
          force_plate_description);

    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_Device) {
      // increase number of devices
      data_descriptions_msg_.n_devices++;

      // Peripheral Device
      sDeviceDescription *p_device =
          data_descriptions_->arrDataDescriptions[i].Data.DeviceDescription;
      ::optitrack_wrapper_ros2_msgs::msg::DeviceDescription device_description =
          ::optitrack_wrapper::GetDeviceDescriptionMessage(p_device);

      // device description add to msg
      data_descriptions_msg_.device_descriptions.push_back(device_description);

    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_Camera) {
      // increase number of cameras
      data_descriptions_msg_.n_cameras++;

      // Camera
      sCameraDescription *p_camera =
          data_descriptions_->arrDataDescriptions[i].Data.CameraDescription;
      ::optitrack_wrapper_ros2_msgs::msg::CameraDescription camera_description =
          ::optitrack_wrapper::GetCameraDescriptionMessage(p_camera);

      // camera description add to msg
      data_descriptions_msg_.camera_descriptions.push_back(camera_description);

    } else {
      RCLCPP_INFO(get_logger(), "Unknown data type.");
      // Unknown
    }
  }
}

void OptitrackWrapper::GetDataDescriptions() {
  // Retrieve Data Descriptions from Motive
  RCLCPP_INFO(get_logger(), "[Client] Requesting Data Descriptions...");

  int i_result = nat_net_client_->GetDataDescriptionList(&data_descriptions_);
  if (i_result != ErrorCode_OK || data_descriptions_ == NULL) {
    RCLCPP_ERROR(get_logger(),
                 "[Client] Unable to retrieve Data Descriptions.");
  } else {
    RCLCPP_INFO(get_logger(), "[Client] Received %d Data Descriptions",
                data_descriptions_->nDataDescriptions);

    if (verbose_data_description_) {
      PrintDataDescriptions();
    }
  }
}

void OptitrackWrapper::PrintDataDescriptions() {
  for (int i = 0; i < data_descriptions_->nDataDescriptions; i++) {
    RCLCPP_INFO(get_logger(), "Data Description # %d (type=%d)", i,
                data_descriptions_->arrDataDescriptions[i].type);
    if (data_descriptions_->arrDataDescriptions[i].type ==
        Descriptor_MarkerSet) {
      // MarkerSet
      sMarkerSetDescription *p_ms =
          data_descriptions_->arrDataDescriptions[i].Data.MarkerSetDescription;
      RCLCPP_INFO(get_logger(), "MarkerSet Name : %s", p_ms->szName);
      for (int i = 0; i < p_ms->nMarkers; i++)
        RCLCPP_INFO(get_logger(), "%s", p_ms->szMarkerNames[i]);
    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_RigidBody) {
      // RigidBody
      sRigidBodyDescription *p_rb =
          data_descriptions_->arrDataDescriptions[i].Data.RigidBodyDescription;
      RCLCPP_INFO(get_logger(), "RigidBody Name : %s", p_rb->szName);
      RCLCPP_INFO(get_logger(), "RigidBody ID : %d", p_rb->ID);
      RCLCPP_INFO(get_logger(), "RigidBody Parent ID : %d", p_rb->parentID);
      RCLCPP_INFO(get_logger(), "Parent Offset : %3.2f,%3.2f,%3.2f",
                  p_rb->offsetx, p_rb->offsety, p_rb->offsetz);

      if (p_rb->MarkerPositions != NULL && p_rb->MarkerRequiredLabels != NULL) {
        for (int marker_idx = 0; marker_idx < p_rb->nMarkers; ++marker_idx) {
          const MarkerData &marker_position = p_rb->MarkerPositions[marker_idx];
          const int marker_required_label =
              p_rb->MarkerRequiredLabels[marker_idx];

          RCLCPP_INFO(get_logger(), "\tMarker #%d:", marker_idx);
          RCLCPP_INFO(get_logger(), "\t\tPosition: %.2f, %.2f, %.2f",
                      marker_position[0], marker_position[1],
                      marker_position[2]);

          if (marker_required_label != 0) {
            RCLCPP_INFO(get_logger(), "\t\tRequired active label: %d",
                        marker_required_label);
          }
        }
      }
    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_Skeleton) {
      // Skeleton
      sSkeletonDescription *p_sk =
          data_descriptions_->arrDataDescriptions[i].Data.SkeletonDescription;
      RCLCPP_INFO(get_logger(), "Skeleton Name : %s", p_sk->szName);
      RCLCPP_INFO(get_logger(), "Skeleton ID : %d", p_sk->skeletonID);
      RCLCPP_INFO(get_logger(), "RigidBody (Bone) Count : %d",
                  p_sk->nRigidBodies);
      for (int j = 0; j < p_sk->nRigidBodies; j++) {
        sRigidBodyDescription *p_rb = &p_sk->RigidBodies[j];
        RCLCPP_INFO(get_logger(), "  RigidBody Name : %s", p_rb->szName);
        RCLCPP_INFO(get_logger(), "  RigidBody ID : %d", p_rb->ID);
        RCLCPP_INFO(get_logger(), "  RigidBody Parent ID : %d", p_rb->parentID);
        RCLCPP_INFO(get_logger(), "  Parent Offset : %3.2f,%3.2f,%3.2f",
                    p_rb->offsetx, p_rb->offsety, p_rb->offsetz);
      }
    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_ForcePlate) {
      // Force Plate
      sForcePlateDescription *p_fp =
          data_descriptions_->arrDataDescriptions[i].Data.ForcePlateDescription;
      RCLCPP_INFO(get_logger(), "Force Plate ID : %d", p_fp->ID);
      RCLCPP_INFO(get_logger(), "Force Plate Serial : %s", p_fp->strSerialNo);
      RCLCPP_INFO(get_logger(), "Force Plate Width : %3.2f", p_fp->fWidth);
      RCLCPP_INFO(get_logger(), "Force Plate Length : %3.2f", p_fp->fLength);
      RCLCPP_INFO(get_logger(),
                  "Force Plate Electrical Center Offset "
                  "(%3.3f, %3.3f, %3.3f)",
                  p_fp->fOriginX, p_fp->fOriginY, p_fp->fOriginZ);
      for (int i_corner = 0; i_corner < 4; i_corner++)
        RCLCPP_INFO(get_logger(),
                    "Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)", i_corner,
                    p_fp->fCorners[i_corner][0], p_fp->fCorners[i_corner][1],
                    p_fp->fCorners[i_corner][2]);
      RCLCPP_INFO(get_logger(), "Force Plate Type : %d", p_fp->iPlateType);
      RCLCPP_INFO(get_logger(), "Force Plate Data Type : %d",
                  p_fp->iChannelDataType);
      RCLCPP_INFO(get_logger(), "Force Plate Channel Count : %d",
                  p_fp->nChannels);
      for (int i_channel = 0; i_channel < p_fp->nChannels; i_channel++)
        RCLCPP_INFO(get_logger(), "\tChannel %d : %s", i_channel,
                    p_fp->szChannelNames[i_channel]);
    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_Device) {
      // Peripheral Device
      sDeviceDescription *p_device =
          data_descriptions_->arrDataDescriptions[i].Data.DeviceDescription;
      RCLCPP_INFO(get_logger(), "Device Name : %s", p_device->strName);
      RCLCPP_INFO(get_logger(), "Device Serial : %s", p_device->strSerialNo);
      RCLCPP_INFO(get_logger(), "Device ID : %d", p_device->ID);
      RCLCPP_INFO(get_logger(), "Device Channel Count : %d",
                  p_device->nChannels);
      for (int i_channel = 0; i_channel < p_device->nChannels; i_channel++)
        RCLCPP_INFO(get_logger(), "\tChannel %d : %s", i_channel,
                    p_device->szChannelNames[i_channel]);
    } else if (data_descriptions_->arrDataDescriptions[i].type ==
               Descriptor_Camera) {
      // Camera
      sCameraDescription *p_camera =
          data_descriptions_->arrDataDescriptions[i].Data.CameraDescription;
      RCLCPP_INFO(get_logger(), "Camera Name : %s", p_camera->strName);
      RCLCPP_INFO(get_logger(), "Camera Position (%3.2f, %3.2f, %3.2f)",
                  p_camera->x, p_camera->y, p_camera->z);
      RCLCPP_INFO(get_logger(),
                  "Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)",
                  p_camera->qx, p_camera->qy, p_camera->qz, p_camera->qw);
    } else {
      RCLCPP_INFO(get_logger(), "Unknown data type.");
      // Unknown
    }
  }
}

void OptitrackWrapper::ConnectOptitrack() {

  RCLCPP_INFO(get_logger(),
              "Trying to connect to Optitrack NatNET SDK at %s ...",
              server_address_.c_str());

  // release previous server
  nat_net_client_->Disconnect();

  // init Client and connect to NatNet server
  int ret_code = nat_net_client_->Connect(nat_net_client_params_);
  if (ret_code != ErrorCode_OK) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to connect to server.  Error code: "
                 "%d. Exiting.",
                 ret_code);
  } else {
    // connection succeeded
    void *p_result;
    int n_bytes = 0;
    ErrorCode ret = ErrorCode_OK;

    // print server info
    memset(&server_description_, 0, sizeof(server_description_));
    ret = nat_net_client_->GetServerDescription(&server_description_);
    if (ret != ErrorCode_OK || !server_description_.HostPresent) {
      RCLCPP_ERROR(get_logger(), "Unable to connect to server. "
                                 "Host not present. Exiting.");
    }

    // set the server description message
    server_description_msg_.host_present = server_description_.HostPresent;
    server_description_msg_.host_computer_name =
        server_description_.szHostComputerName;
    for (int i = 0; i < 4; i++) {
      server_description_msg_.host_computer_address[i] =
          server_description_.HostComputerAddress[i];
      server_description_msg_.host_app_version[i] =
          server_description_.HostAppVersion[i];
      server_description_msg_.nat_net_version[i] =
          server_description_.NatNetVersion[i];
      server_description_msg_.connection_multicast_address[i] =
          server_description_.ConnectionMulticastAddress[i];
    }
    server_description_msg_.host_app = server_description_.szHostApp;
    server_description_msg_.high_res_clock_frequency =
        server_description_.HighResClockFrequency;
    server_description_msg_.connection_info_valid =
        server_description_.bConnectionInfoValid;
    server_description_msg_.connection_data_port =
        server_description_.ConnectionDataPort;
    server_description_msg_.connection_multicast =
        server_description_.ConnectionMulticast;

    // display info
    RCLCPP_INFO(get_logger(), "[Client] Server application info:");
    RCLCPP_INFO(get_logger(), "Application: %s (ver. %d.%d.%d.%d)",
                server_description_.szHostApp,
                server_description_.HostAppVersion[0],
                server_description_.HostAppVersion[1],
                server_description_.HostAppVersion[2],
                server_description_.HostAppVersion[3]);
    RCLCPP_INFO(get_logger(), "NatNet Version: %d.%d.%d.%d",
                server_description_.NatNetVersion[0],
                server_description_.NatNetVersion[1],
                server_description_.NatNetVersion[2],
                server_description_.NatNetVersion[3]);
    RCLCPP_INFO(get_logger(), "Client IP:%s",
                nat_net_client_params_.localAddress);
    RCLCPP_INFO(get_logger(), "Server IP:%s",
                nat_net_client_params_.serverAddress);
    RCLCPP_INFO(get_logger(), "Server Name:%s",
                server_description_.szHostComputerName);

    // get mocap frame rate
    ret = nat_net_client_->SendMessageAndWait("FrameRate", &p_result, &n_bytes);
    if (ret == ErrorCode_OK) {
      float f_rate = *((float *)p_result);
      RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f", f_rate);
    } else
      RCLCPP_INFO(get_logger(), "Error getting frame rate.");

    // get # of analog samples per mocap frame of data
    ret = nat_net_client_->SendMessageAndWait("AnalogSamplesPerMocapFrame",
                                              &p_result, &n_bytes);
    if (ret == ErrorCode_OK) {
      analog_samples_per_mocap_frame_ = *((int *)p_result);
      RCLCPP_INFO(get_logger(), "Analog Samples Per Mocap Frame : %d",
                  analog_samples_per_mocap_frame_);
    } else
      RCLCPP_ERROR(get_logger(), "Error getting Analog frame rate.");
  }
}

void OptitrackWrapper::DeclareRosParameters() {
  declare_parameter<::std::string>("connection_type", "Multicast");
  declare_parameter<::std::string>("server_address", "000.000.000.000");
  declare_parameter<::std::string>("local_address", "000.000.000.000");
  declare_parameter<::std::string>("multicast_address", "000.000.000.000");
  declare_parameter<uint16_t>("server_command_port", 0);
  declare_parameter<uint16_t>("server_data_port", 0);
  declare_parameter<bool>("verbose_data_description", true);
  declare_parameter<bool>("verbose_frame", false);
  declare_parameter<::std::string>("topic_frame_data", "frame_data");
}

void OptitrackWrapper::InitializeRosParameters() {
  get_parameter<::std::string>("connection_type", connection_type_);
  get_parameter<::std::string>("server_address", server_address_);
  get_parameter<::std::string>("local_address", local_address_);
  get_parameter<::std::string>("multicast_address", multicast_address_);
  get_parameter<uint16_t>("server_command_port", server_command_port_);
  get_parameter<uint16_t>("server_data_port", server_data_port_);
  get_parameter<bool>("verbose_data_description", verbose_data_description_);
  get_parameter<bool>("verbose_frame", verbose_frame_);
  get_parameter<::std::string>("topic_frame_data", topic_frame_data_);
}

void OptitrackWrapper::InitializeClient() {
  // print NatNet version info
  unsigned char ver[4];
  NatNet_GetVersion(ver);
  RCLCPP_INFO(this->get_logger(), "NatNet Client (NatNet ver. %d.%d.%d.%d)",
              ver[0], ver[1], ver[2], ver[3]);

  // create client
  nat_net_client_ = new NatNetClient();
  nat_net_client_->SetFrameReceivedCallback(ProcessFrameCallback, this);
}

void OptitrackWrapper::InitializeClientParams() {
  if (connection_type_ == "Multicast") {
    nat_net_client_params_.connectionType =
        ConnectionType::ConnectionType_Multicast;
    nat_net_client_params_.multicastAddress = multicast_address_.c_str();
  } else if (connection_type_ == "Unicast") {
    nat_net_client_params_.connectionType =
        ConnectionType::ConnectionType_Unicast;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown connection type -- "
                               "options are Multicast, Unicast");
    rclcpp::shutdown();
  }

  nat_net_client_params_.serverAddress = server_address_.c_str();
  nat_net_client_params_.localAddress = local_address_.c_str();
  nat_net_client_params_.serverCommandPort = server_command_port_;
  nat_net_client_params_.serverDataPort = server_data_port_;
}

} // namespace optitrack_wrapper
