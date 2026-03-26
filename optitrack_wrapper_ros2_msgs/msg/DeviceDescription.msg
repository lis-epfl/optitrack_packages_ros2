# used for order, and for identification in the data stream
int32 id

# device name as appears in Motive
string name

# for unique device identification
string serial_no

# device 'type' code
int32 device_type

# channel data type code
int32 channel_data_type

# number of currently enbaled/active channels (signals)
int32 n_channels

# channel
string[] channel_names
