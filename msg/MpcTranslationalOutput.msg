std_msgs/Header header
uint64 trajectory_id
uint64 sequence

# Exact current feedback state used as x0=[p,v,a] by all three solvers.
float64[3] measured_position
float64[3] measured_velocity
float64[3] measured_acceleration
# First reference point on the solver grid.
float64[3] sampled_reference_position
float64[3] sampled_reference_velocity
float64[3] sampled_reference_acceleration

# Optimizer input u0 for X/Y/Z.
float64[3] control_input
# First predicted physical acceleration state a[k+1]. Diagnostic only.
float64[3] first_predicted_acceleration
# QP prediction:
# [step][axis][p,v,a], 26 * 3 * 3 values.
float64[] predicted_states
# Per-axis ADMM diagnostics ordered X/Y/Z.
int32[3] solver_iterations
uint8[3] solver_status
float64[3] solver_primal_residual
float64[3] solver_dual_residual
float64[3] solver_primal_tolerance
float64[3] solver_dual_tolerance
bool[3] recovery_constraint_active
float64 solve_time_seconds
bool solver_deadline_missed
bool recovery_command_active
uint8 failure_reason
bool valid

uint8 SOLVER_SUCCESS=0
uint8 SOLVER_INVALID_INPUT=1
uint8 SOLVER_INFEASIBLE_BOUNDS=2
uint8 SOLVER_FACTORIZATION_FAILURE=3
uint8 SOLVER_DEADLINE_EXCEEDED=4
uint8 SOLVER_MAX_ITERATIONS=5
uint8 SOLVER_NON_FINITE_OUTPUT=6
uint8 SOLVER_OUTPUT_LIMIT=7
