# Low-level command actually sent to the base Mega (#1), published by
# rover_base for introspection. The real control input is geometry_msgs/Twist
# on /cmd_vel, reinterpreted per the active rover_msgs/DriveMode - see
# rover_base/kinematics.py.
std_msgs/Header header
uint8 drive_mode          # mirrors rover_msgs/DriveMode constants (ACKERMANN/POINT_TURN/STOP)
int32[6] wheel_throttle   # -1000..1000 per wheel, order: FL,FR,ML,MR,RL,RR
int32[4] steer_decideg    # commanded steering angle *10 deg, order: FL,FR,RL,RR (ML/MR fixed)
