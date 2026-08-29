# Command for the mast Uno (#3): 2-axis head (yaw/pitch) plus the mast
# erect/stow lift axis.
std_msgs/Header header
int32 head_yaw_decideg
int32 head_pitch_decideg
int8 lift_mode          # -1 = stow (transport/horizontal), 0 = hold, 1 = erect (service/vertical)
bool driver_enable       # energize/de-energize the yaw/pitch TB6600 drivers - only takes effect once homing and the post-calibration sequence have both finished, see mast_protocol.py
