std_msgs/Header header
uint64 trajectory_id
uint64 sequence
uint64 publisher_steady_timestamp_ns
uint64 timer_sequence
uint64 timer_expected_fire_steady_timestamp_ns
uint64 timer_actual_start_steady_timestamp_ns
uint64 timer_callback_end_steady_timestamp_ns

# Feasible acceleration command after the explicit coupled force stage, in m/s^2.
float64[3] desired_acceleration_m_s2
# Desired inertial force in ENU, Newtons.
float64[3] desired_force_world_n
# Raw independent-axis actuator acceleration command and force before coupled
# feasibility shaping. This is optimizer input u, not modeled state a[k+1].
float64[3] raw_desired_acceleration_m_s2
float64[3] feasible_desired_acceleration_m_s2
float64[3] raw_desired_force_world_n
float64[3] feasible_desired_force_world_n
float64 raw_force_norm_n
float64 feasible_force_norm_n
float64 force_limit_n
float64 feasibility_correction_norm_m_s2
bool feasibility_constraint_active
# Quaternion order is [w, x, y, z], body FLU to world ENU.
float64[4] desired_attitude_wxyz
# Row-major rotation matrix, world ENU from body FLU.
float64[9] desired_rotation_world_from_body
# Desired world heading used to construct desired_attitude_wxyz, ENU radians.
float64 desired_yaw_enu_rad
# Positive about +Z_ENU, radians per second. The PX4 adapter maps this to
# yaw_sp_move_rate in NED; it is not a body-rate command.
float64 desired_yaw_rate_enu_rad_s
bool desired_yaw_rate_valid
# Collective force sent to the attitude adapter, Newtons.
float64 desired_thrust_force_n
float64 tilt_angle_rad
bool math_valid
bool active_control_ready
bool valid

uint8 FAILURE_NONE=0
uint8 FAILURE_INVALID_PARAMETERS=1
uint8 FAILURE_NON_FINITE_INPUT=2
uint8 FAILURE_INVALID_INPUT=3
uint8 FAILURE_DEGENERATE_FORCE=4
uint8 FAILURE_TILT_LIMIT=5
uint8 FAILURE_DEGENERATE_HEADING_BASIS=6
uint8 FAILURE_INVALID_ROTATION=7
uint8 FAILURE_RATE_COMMAND_INVALID=8
uint8 FAILURE_NON_FINITE_OUTPUT=9
uint8 FAILURE_FEASIBILITY_INVALID=10
uint8 FAILURE_INVALID_COLLECTIVE_PROJECTION=11
uint8 failure_reason
