std_msgs/Header header
uint64 sequence

# Feasible command after coupled force limiting.
float64[3] desired_acceleration_m_s2
float64[3] desired_force_world_n
float64 raw_force_norm_n
float64 feasible_force_norm_n
float64 force_limit_n
float64 feasibility_correction_norm_m_s2
bool feasibility_constraint_active
# Quaternion order is [w, x, y, z], body FLU to world ENU.
float64[4] desired_attitude_wxyz
float64 desired_thrust_force_n
float64 tilt_angle_rad
