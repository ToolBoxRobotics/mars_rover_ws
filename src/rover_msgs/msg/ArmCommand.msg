# Target joint positions for the 5-axis arm Mega (#2), in motor steps
# from each joint's home position (established by the calibration/limit
# switches at startup homing).
std_msgs/Header header
int32[5] joint_target_steps  # J1..J5
bool enable                  # false de-energizes all A4988 drivers (free-spin)
