std_msgs/Header header
uint64 trajectory_id
uint64 sequence

# Optimizer input u0 for X/Y/Z. For Z this is not equal to a[k+1].
float64[3] control_input
# First predicted state acceleration a[k+1] for X/Y/Z.
float64[3] first_predicted_acceleration
# Flattened prediction: [step][axis][p,v,a], 26 * 3 * 3 values.
float64[] predicted_states
int32[3] solver_iterations
float64 solve_time_seconds
bool solver_success
bool valid
