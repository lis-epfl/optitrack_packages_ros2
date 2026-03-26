# RigidBody identifier.
# For rigid body assets, this is the StreamingID value
# For skeleton assets, this combines both skeleton ID (high-bit) and Bone ID (low-bit).
int32 id

# pose
geometry_msgs/Pose pose

# mean measure-to-solve deviation
float32 mean_error

# RigidBody was successfully tracked in this frame
bool tracking_valid
