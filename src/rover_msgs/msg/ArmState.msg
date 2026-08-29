# Feedback from the arm Mega (#2).
std_msgs/Header header
int32[5] joint_position_steps
bool[5] limit_switch_triggered
bool[5] joint_homed           # per-joint homed status - a joint can be individually re-homed without touching the others
bool homed                    # true once EVERY joint is homed (all(joint_homed)) - computed by rover_arm_bridge, not sent over the wire; convenience for consumers (e.g. MoveIt's trajectory_action_server) that only care about "is the whole arm ready"
int32 supply_voltage_mv       # main battery voltage, from an FZ0430 sensor on analog pin A0
int32 board_temperature_decic # board/enclosure temperature in tenths of a degree Celsius, from a DS18B20 on 1-Wire - -9999 ("sensor not found") if the sensor didn't respond on its most recent read
uint8 fan_duty_percent        # 0-100, cooling fan PWM duty cycle - entirely automatic (a function of board_temperature_decic), never operator-commanded; 0 just means the fan is currently off, not necessarily broken
bool estop_active             # true while the emergency stop is latched - the firmware's own authoritative state (see arm_mega2.ino's handleEmergencyStop()), not inferred from what was last commanded; blocks all new joint/preset motion until explicitly cleared
