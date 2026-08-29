# Feedback from the base Mega (#1): the two fixed middle wheels' (ML,
# MR) quadrature encoders. Only these two are physically encoded - see
# rover_base/rover_base/odometry.py for why they're the only pair
# useful for wheel odometry (rolling axis never changes with corner
# steering, unlike FL/FR/RL/RR). Order: ML, MR.
std_msgs/Header header
int32[2] encoder_ticks       # cumulative ticks, order: ML,MR
int32[2] encoder_delta_ticks # ticks since previous message (for odometry)
int32 drive_voltage_mv        # drive motors' own supply rail voltage, from an FZ0430 sensor on analog pin A0 - previously a generic "supply_voltage_mv" before a second FZ0430 was added; renamed to be specific now that there are two independent rails to report
int32 steering_voltage_mv     # steering servos' own supply rail voltage, from a second, identically-configured FZ0430 on analog pin A1
int32 board_temperature_decic # board/enclosure temperature in tenths of a degree Celsius, from a DS18B20 on 1-Wire - -9999 ("sensor not found") if the sensor didn't respond on its most recent read
uint8 fan_duty_percent        # 0-100, cooling fan PWM duty cycle - entirely automatic (a function of board_temperature_decic), never operator-commanded; 0 just means the fan is currently off, not necessarily broken
