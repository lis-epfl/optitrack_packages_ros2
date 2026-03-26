# used for order, and for identification in the data stream
int32 id

# for unique plate identification
string serial_no

# plate physical width (manufacturer supplied)
float32 width

# plate physical length (manufacturer supplied)
float32 length

# electrical center offset (from electrical center to geometric center-top of force plate) (manufacturer supplied)
geometry_msgs/Point origin

# force plate calibration matrix (for raw analog voltage channel type only); the dimension is [12][12] to [144] where [0][0] is [0] and [0][1] is [1] and [1][0] is [12] and [1][1] is [13]
float32[144] cal_mat

# plate corners, in world (aka Mocap System) coordinates, clockwise from plate +x,+y (refer to C3D spec for details); the dimension is [4][3] to [12] with the same method as cal_mat
float32[12] corners

# force plate 'type' (refer to C3D spec for details)
int32 plate_type

# 0=Calibrated force data, 1=Raw analog voltages
int32 channel_data_type

# number of channels
int32 n_channels

# channel names
string[] channel_names
