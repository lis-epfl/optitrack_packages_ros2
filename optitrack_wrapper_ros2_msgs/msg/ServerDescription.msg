# host is present and accounted for
bool host_present

# host computer name
string host_computer_name

# host IP address
uint8[4] host_computer_address

# name of host app
string host_app

# version of host app
uint8[4] host_app_version

# host app's version of NatNet
uint8[4] nat_net_version

# host's high resolution clock frequency (ticks per second)
uint64 high_res_clock_frequency

# if the server predates NatNet 3.0, this will be false, and the other Connection* fields invalid.
bool connection_info_valid

# the data port this server is configured to use.
uint16 connection_data_port

# whether this server is streaming in multicast. If false, connect in unicast instead.
bool connection_multicast

# the multicast group address to use for a multicast connection.
uint8[4] connection_multicast_address
