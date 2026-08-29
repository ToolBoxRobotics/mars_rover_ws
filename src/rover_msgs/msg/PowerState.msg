# Feedback from the power/environmental monitoring Uno (#6). Unlike
# every other *State message in this package, there is no matching
# *Command message for this board - it commands nothing, it only
# reports. See power_uno6.ino's own header comment for why.
std_msgs/Header header
int32 battery1_voltage_mv        # first 24V battery, from its own INA226 (behind a TCA9548A mux, channel 0) - bus voltage range 0-36V, genuine headroom above what a nominal 24V pack can realistically reach (see README's "Explicit assumptions")
int32 battery1_current_ma        # first battery's own current draw, from that same INA226 - signed; see power_uno6.ino's SHUNT RESISTOR WARNING before trusting this at anything beyond a rough reading
int32 battery2_voltage_mv        # second 24V battery, from its own INA226 (mux channel 1) - same sensor, same caveats as battery1_voltage_mv
int32 battery2_current_ma        # second battery's own current draw, from that same INA226 - signed, same caveat as battery1_current_ma
int32 computer_temperature_decic # onboard computer temperature in tenths of a degree Celsius, from a DS18B20 on 1-Wire - -9999 ("sensor not found") if the sensor didn't respond on its most recent read
uint8 fan_duty_percent           # 0-100, cooling fan PWM duty cycle for the onboard computer - entirely automatic (a function of computer_temperature_decic), never operator-commanded; 0 just means the fan is currently off, not necessarily broken

