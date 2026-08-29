# Feedback from the antenna gimbal Uno (#5).
std_msgs/Header header
int32 azimuth_decideg
int32 elevation_decideg
bool azimuth_limit_triggered    # true if the azimuth calibration switch is currently triggered
bool elevation_limit_triggered  # true if the elevation calibration switch is currently triggered
bool homed                       # true once both axes have found their calibration switch (each switch is that axis's own operational minimum, so no separate move-to-center step is needed the way the mast's does)
int32 supply_voltage_mv          # main battery voltage, from an FZ0430 sensor on analog pin A0
bool driver_enabled               # true if the shared TB6600 enable pin is currently energized - starts true (homing needs it), same convention as MastState's own driver_enabled field
int32 board_temperature_decic     # board/enclosure temperature in tenths of a degree Celsius, from a DS18B20 on 1-Wire - -9999 ("sensor not found") if the sensor didn't respond on its most recent read
uint8 fan_duty_percent            # 0-100, cooling fan PWM duty cycle - entirely automatic (a function of board_temperature_decic), never operator-commanded; 0 just means the fan is currently off, not necessarily broken
