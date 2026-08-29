# Command for the microscope Uno (#4) at the arm's 3rd wrist joint.
std_msgs/Header header
int32 focus_target_steps   # combined focus/zoom stepper target (24BYJ-48 + DRV8825, wired 4-wire bipolar)
uint8 led_pwm               # 0..255 dimmable 5V LED brightness
bool cover_open              # commands the SG90 sliding lens cover
bool driver_enable           # true to energize the DRV8825 (enable), false to de-energize it (disable / free-spin)
