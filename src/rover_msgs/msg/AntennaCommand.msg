# Command for the antenna gimbal Uno (#5): 2-axis (azimuth/elevation)
# high-gain-antenna pointing mechanism.
std_msgs/Header header
int32 azimuth_decideg      # G1 - primary axis, normal to the deck. Operational range 15-285 deg (see rover_antenna's antenna_protocol.py)
int32 elevation_decideg    # G2 - secondary axis, parallel to the deck. Operational range 0-180 deg
bool driver_enable          # energize/de-energize the shared TB6600 enable pin - only takes effect once homed is true, same reasoning as MastCommand's own driver_enable field
