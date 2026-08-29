# Feedback from the microscope Uno (#4).
std_msgs/Header header
int32 focus_position_steps
uint8 led_pwm
bool cover_open
bool homed
bool driver_enabled
int32 board_temperature_decic # board/enclosure temperature in tenths of a degree Celsius, from a DS18B20 on 1-Wire - -9999 ("sensor not found") if the sensor didn't respond on its most recent read
uint8 fan_duty_percent        # 0-100, cooling fan PWM duty cycle - entirely automatic (a function of board_temperature_decic), never operator-commanded; 0 just means the fan is currently off, not necessarily broken
