std_msgs/Header header
uint64 sequence

# Specific force is force divided by vehicle mass, so this contract works
# without an airframe mass value.
float64[3] desired_acceleration_m_s2
float64[3] desired_specific_force_world_m_s2
# Quaternion order is [w, x, y, z], body FLU to world ENU.
float64[4] desired_attitude_wxyz
float64 desired_collective_specific_force_m_s2
float64 tilt_angle_rad
bool recovery_active
