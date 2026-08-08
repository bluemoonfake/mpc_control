std_msgs/Header header
uint64 trajectory_id
uint64 sequence

# MRS-style acceleration command: [u_x, u_y, u_z] from M2, in m/s^2.
float64[3] desired_acceleration_m_s2
# Desired inertial force in ENU, Newtons.
float64[3] desired_force_world_n
# Quaternion order is [w, x, y, z], body FLU to world ENU.
float64[4] desired_attitude_wxyz
# Row-major rotation matrix, world ENU from body FLU.
float64[9] desired_rotation_world_from_body
# Projection of desired force onto current body +Z, Newtons.
float64 desired_thrust_force_n
float64 tilt_angle_rad
float64[3] orientation_error
float64[3] desired_body_rate_rad_s
float64[3] body_rate_error_rad_s
# MRS attitude-rate control-group action, retained for source-parity diagnostics.
float64[3] control_group_action
# PX4-normalized rate-controller command in body FLU coordinates. Dimensionless;
# not physical torque and not N*m.
float64[3] normalized_torque_command_flu
bool normalized_torque_saturated
bool math_valid
bool active_control_ready
bool valid
