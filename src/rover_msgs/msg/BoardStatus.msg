# Generic connection-health status for one Arduino board, published by
# every bridge node (base / arm / mast / microscope) on its own topic.
std_msgs/Header header
string board_name          # e.g. "base_mega1", "arm_mega2"
string port                 # serial device path actually opened, e.g. /dev/rover/base
bool connected               # true once the port is open and frames are flowing
int32 rx_frame_count         # total valid frames received since node start
int32 checksum_error_count   # frames dropped for bad checksum / malformed data
int32 reconnect_count        # number of times the serial port was reopened
float32 last_rx_age_sec      # seconds since the last valid frame (informational)
