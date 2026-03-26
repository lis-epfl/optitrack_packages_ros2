# RigidBody name
string name

# RigidBody identifier: Streaming ID value for rigid body assets, and Bone index value for skeleton rigid bodies.
int32 id

# ID of parent Rigid Body (in case hierarchy exists; otherwise -1)
int32 parent_id

# offset position relative to parent
geometry_msgs/Point offset

# number of markers associated with this rigid body
int32 n_markers

# array of marker locations ( [n_markers][3] )
geometry_msgs/Point[] marker_positions

# array of expected marker active labels - 0 if not specified. ( [n_markers] )
int32[] marker_required_labels

# array of marker names
string[] marker_names
