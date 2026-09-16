std_msgs/Header header
uint64 sequence

# Specific force is force divided by vehicle mass, so this contract works
# without an airframe mass value.
float64[3] desired_acceleration_m_s2
float64[3] desired_specific_force_world_m_s2
# Quaternion order is [w, x, y, z], body FLU to world ENU.
float64[4] desired_attitude_wxyz
# Positive yaw rate follows the project's ENU convention. The PX4 adapter
# converts it to NED before writing VehicleAttitudeSetpoint.yaw_sp_move_rate.
float64 desired_yaw_rate_rad_s
float64 desired_collective_specific_force_m_s2
float64 tilt_angle_rad
bool recovery_active
