# header stamp contains the time of capture
std_msgs/Header header

# frame number
int32 frame

# markers
MarkerBase[] markers

# rigid body info
RigidBodyBase rigid_body

# pipeline total latency in ms just before publishing
float64 latency_ms
