# TMPC state order:
# [px, py, pz, vx, vy, vz, roll, pitch, yaw, yaw_rate,
#  collective_specific_force]
std_msgs/Header header
uint64 trajectory_id
uint64 sequence
float64[11] measured_state
float64[11] first_reference_state

# TMPC input order:
# [roll_command, pitch_command, yaw_command, collective_specific_force_command]
float64[4] first_reference_input
# Command directly produced by the solver or recovery law before the local
# roll/pitch low-pass filter.
float64[4] raw_control_input
# Command published to the attitude adapter after the local low-pass filter.
# `control_input` is kept as this value for compatibility with existing tools.
float64[4] filtered_control_input
float64[4] control_input
# Latest command confirmed by the attitude adapter after its safety limiter.
# It can belong to the preceding controller cycle; consult the availability and
# age fields before using it for analysis.
float64[4] applied_control_input
float64 applied_command_age_seconds
bool applied_command_available

# Flattened predictions: state stages (N+1)*11 and input stages N*4.
float64[] predicted_states
float64[] predicted_inputs

uint8 solver_status
int32 solver_iterations
# Acados-reported solve time. The remaining values are wall-clock timings for
# the controller-side preparation, Acados call, diagnostics/output validation,
# and total solve() call respectively.
float64 solve_time_seconds
float64 preparation_time_seconds
float64 acados_wall_time_seconds
float64 postprocessing_time_seconds
float64 acados_metadata_time_seconds
float64 diagnostics_time_seconds
float64 sqp_statistics_time_seconds
float64 prediction_read_time_seconds
float64 constraint_validation_time_seconds
float64 result_finalization_time_seconds
float64 postprocessing_unattributed_time_seconds
float64 end_to_end_time_seconds
# Age of the state measurement at the moment the output is published.
# Measures total sensor-to-output latency including DDS transport and solve.
float64 setpoint_age_seconds
# Age of the reference trajectory at the moment the output is published.
float64 reference_age_seconds
float64 max_constraint_violation
bool solver_available
bool valid
bool deadline_missed
bool recovery_active
string failure_reason

uint8 SOLVER_SUCCESS=0
uint8 SOLVER_INVALID_INPUT=1
uint8 SOLVER_DEPENDENCY_UNAVAILABLE=2
uint8 SOLVER_NOT_INITIALIZED=3
uint8 SOLVER_INFEASIBLE=4
uint8 SOLVER_DEADLINE_EXCEEDED=5
uint8 SOLVER_NUMERICAL_FAILURE=6
uint8 SOLVER_MAXIMUM_ITERATIONS=7
uint8 SOLVER_MINIMUM_STEP=8
