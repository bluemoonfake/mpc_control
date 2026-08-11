std_msgs/Header header
uint64 trajectory_id
uint64 sequence
uint64 publisher_steady_timestamp_ns
uint64 timer_sequence
uint64 timer_expected_fire_steady_timestamp_ns
uint64 timer_actual_start_steady_timestamp_ns
uint64 timer_callback_end_steady_timestamp_ns

# Exact current feedback state used as x0=[p,v,a] by all three solvers.
float64[3] measured_position
float64[3] measured_velocity
float64[3] measured_acceleration
# First reference point on the actual solver grid, at t+dt_first.
float64 first_reference_time_offset_seconds
float64[3] sampled_reference_position
float64[3] sampled_reference_velocity
float64[3] sampled_reference_acceleration

# Optimizer input u0 for X/Y/Z. In general this is not a physical
# acceleration; a[k+1] = alpha*a[k] + (1-alpha)*u[k].
float64[3] control_input
# First predicted physical acceleration state a[k+1] for X/Y/Z. M3 consumes
# this field at the force-command boundary.
float64[3] first_predicted_acceleration
# Flattened prediction: [step][axis][p,v,a], 26 * 3 * 3 values.
float64[] predicted_states
int32[3] solver_iterations
float64 solve_time_seconds
bool solver_success
bool valid
