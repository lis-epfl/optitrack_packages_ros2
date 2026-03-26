# time of publication
builtin_interfaces/Time stamp

# frame number
int32 frame

# unlabeled markers id:
# unlabeled markers can change IDs any time the trajectory is broken
int32[] id

# unlabeled markers position
geometry_msgs/Point[] positions

# pipeline total latency in ms just before publishing
float64 latency_ms
