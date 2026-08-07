# Measured/external state used for activation and stale/reset checks.
# This message is not an estimator output produced by the MPC wrapper.
std_msgs/Header header
uint64 sequence
float64[3] position
float64[3] velocity
float64[3] acceleration
float64 yaw
float64 yaw_rate
bool valid
bool position_valid
bool velocity_valid
bool acceleration_valid
bool yaw_valid
uint32 reset_counter
bool reset_counter_valid
