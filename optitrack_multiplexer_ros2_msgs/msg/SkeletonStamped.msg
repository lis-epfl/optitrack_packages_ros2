# header stamp contains the time of capture
std_msgs/Header header

# frame number
int32 frame

# skeleton id
int32 id

# markers
MarkerBase[] markers

# rigid bodies poses
RigidBodyBase[] rigid_bodies

# pipeline total latency in ms just before publishing
float64 latency_ms
