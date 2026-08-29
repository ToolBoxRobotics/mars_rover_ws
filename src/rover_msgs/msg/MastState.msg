# Feedback from the mast Uno (#3).
std_msgs/Header header
int32 head_yaw_decideg
int32 head_pitch_decideg
uint8 STATE_UNKNOWN=0
uint8 STATE_TRANSPORT=1     # mast stowed horizontal, at stowed limit switch
uint8 STATE_SERVICE=2       # mast erect vertical, at erect limit switch
uint8 STATE_MOVING=3
uint8 lift_state
bool yaw_limit_triggered    # true if the yaw calibration switch is currently triggered
bool pitch_limit_triggered  # true if the pitch calibration switch is currently triggered
bool homed                  # true once the yaw/pitch calibration sequence has completed
int32 supply_voltage_mv     # main battery voltage, from an FZ0430 sensor on analog pin A0
bool driver_enabled         # true if the yaw/pitch TB6600 drivers are currently energized - starts true, goes false once the automatic post-calibration sequence finishes
int32 board_temperature_decic # board/enclosure temperature in tenths of a degree Celsius, from a DS18B20 on 1-Wire - -9999 ("sensor not found") if the sensor didn't respond on its most recent read
uint8 fan_duty_percent        # 0-100, cooling fan PWM duty cycle - entirely automatic (a function of board_temperature_decic), never operator-commanded; 0 just means the fan is currently off, not necessarily broken
