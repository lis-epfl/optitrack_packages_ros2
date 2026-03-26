# MarkerSet name
string name

# number of markers in MarkerSet
int32 n_markers

# array of marker data ( [n_markers][3] )
# marker positions x, y, z
geometry_msgs/Point[] markers
