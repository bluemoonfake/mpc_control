std_msgs/Header header
uint64 trajectory_id
uint64 sequence

# Current feedback state used as x0=[p,v,a] by the selected backend.
# XY acceleration is the model-consistent observer estimate; Z is measured.
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
# Legacy per-axis diagnostics ordered X/Y/Z; retained for shadow comparison.
int32[3] solver_iterations
uint8[3] solver_status
float64[3] solver_primal_residual
float64[3] solver_dual_residual
float64[3] solver_primal_tolerance
float64[3] solver_dual_tolerance
bool[3] recovery_constraint_active
# Coupled 3D QP diagnostics. In shadow mode the legacy command remains active.
bool coupled_solver_ran
bool coupled_solver_valid
bool coupled_control_active
int32 coupled_solver_iterations
uint8 coupled_solver_status
float64 coupled_primal_residual
float64 coupled_dual_residual
float64 coupled_primal_tolerance
float64 coupled_dual_tolerance
float64 coupled_velocity_slack
float64 coupled_acceleration_slack
float64 coupled_max_constraint_violation
float64 coupled_max_speed_xy
float64 coupled_max_acceleration_xy
float64 coupled_max_tilt_rad
float64 coupled_max_collective_specific_force_m_s2
float64 coupled_solve_time_seconds
float64 shadow_control_difference_norm
uint32 shadow_admission_samples
bool shadow_admission_ready
uint8 active_backend
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

uint8 BACKEND_LEGACY=0
uint8 BACKEND_COUPLED_SHADOW=1
uint8 BACKEND_COUPLED=2
