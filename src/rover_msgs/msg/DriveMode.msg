# Selects the base's steering geometry. Published to rover_base/drive_mode;
# rover_base_bridge holds the last value received (no staleness timeout -
# unlike velocity, a "stuck" mode selection is not itself unsafe) and
# reinterprets incoming geometry_msgs/Twist on /cmd_vel differently
# depending on which mode is active - see rover_base/kinematics.py.
uint8 ACKERMANN=0    # normal driving: Twist.linear.x + Twist.angular.z, ICR-tangent corner steering
uint8 POINT_TURN=1    # rotate about the rover's own center: Twist.angular.z only, linear forced to zero
uint8 STOP=2           # hard override: all wheel throttle and steer angles forced to zero, Twist ignored
uint8 mode
