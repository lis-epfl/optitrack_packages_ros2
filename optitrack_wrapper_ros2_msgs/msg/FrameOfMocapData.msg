# timestamp: time immediately before publishing; used to compute ROS2 communication latency
builtin_interfaces/Time time_publishing

# host defined frame number
int32 frame

# number of marker sets in this frame of data
int32 n_marker_sets

# MarkerSet data
MarkerSetData[] marker_set_data

# number of undefined markers
int32 n_other_markers

# undefined markers data
geometry_msgs/Point[] other_markers

# number of rigid bodies
int32 n_rigid_bodies

# rigid body data
RigidBodyData[] rigid_bodies

# number of skeletons
int32 n_skeletons

# skeleton data
SkeletonData[] skeletons

# number of labeled markers
int32 n_labeled_markers

# labeled marker data (labeled markers not associated with a "MarkerSet")
Marker[] labeled_markers

# number of force plates
int32 n_force_plates

# force plate data
ForcePlateData[] force_plates

# number of  devices
int32 n_devices

# device data
DeviceData[] devices

# SMPTE timecode (if available)
string timecode

# timestamp since software start ( software timestamp )
float64 timestamp

# Given in host's high resolution ticks (from e.g. QueryPerformanceCounter).
uint64 camera_mid_exposure_timestamp

# Given in host's high resolution ticks (from e.g. QueryPerformanceCounter).
uint64 camera_data_received_timestamp

# Given in host's high resolution ticks (from e.g. QueryPerformanceCounter).
uint64 transmit_timestamp

# latency between camera mid exposure and camera data recieved (in ms)
float64 latency_camera_mid_exposure_to_data_received_ms

# latency between camera camera data recieved and transmission (in ms)
float64 latency_camera_data_received_to_transmission_ms

# latency to transmit to NatNet client (in ms)
float64 latency_transmission_to_client_ms

# latency of data handling before transmitting over ROS (in ms)
float64 latency_data_handling_ms

# recording
bool recording

# model list changed
bool model_list_changed

# Live/Edit mode (0=Live, 1=Edit)
bool live_edit_mode
