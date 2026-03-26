# unique identifier:
# for active markers (LED light), this is the Active ID. For passive markers (reflective spheres), this is the PointCloud assigned ID.
# for legacy assets that are created prior to 2.0, this is both AssetID (High-bit) and Member ID (Lo-bit)
int32 id

# model id
int32 model_id

# marker id
int32 marker_id

# position
geometry_msgs/Point position

# marker size
float32 size


# host defined parameters.
# 0 : Occluded: if the marker is not visible by the cameras
bool occluded

# 1 : PointCloudSolved: if it has been triangulated by the cameras; reported position provided by point cloud solve
bool point_cloud_solved

# 2 : ModelFilled: if it is occluded and has been deduced from the model; reported position provided by model solve
bool model_filled

# 3 : HasModel: belongs to a model (marker set, rigid body, skeleton)
bool has_model

# 4 : Unlabeled: is not labeled and doesn't belong to a model
bool unlabeled

# 5 : Active: is active i.e. is an LED light instead of a reflector.
bool active

# 6 : Established (unknown - dig deeper in documentation)
bool established

# 7 : Measurement (unknown - dig deeper in documentation)
bool measurement

# marker error residual, in mm/ray i.e. the smallest distance between the ray and the final triangulated position of the marker
float32 residual
