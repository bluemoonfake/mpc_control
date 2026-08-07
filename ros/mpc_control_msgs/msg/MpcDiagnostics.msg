# Operational status and failure evidence; never a control input.
std_msgs/Header header

uint8 lifecycle_state
uint8 LIFECYCLE_UNCONFIGURED=0
uint8 LIFECYCLE_INACTIVE=1
uint8 LIFECYCLE_ACTIVE=2
uint8 LIFECYCLE_ERROR=3

uint8 failure_reason
uint8 FAILURE_NONE=0
uint8 FAILURE_INVALID_CONFIGURATION=1
uint8 FAILURE_INVALID_REFERENCE=2
uint8 FAILURE_INVALID_STATE=3
uint8 FAILURE_STALE_REFERENCE=4
uint8 FAILURE_STALE_STATE=5
uint8 FAILURE_OUT_OF_ORDER=6
uint8 FAILURE_TIME_JUMP=7
uint8 FAILURE_SOLVER_X=8
uint8 FAILURE_SOLVER_Y=9
uint8 FAILURE_SOLVER_Z=10
uint8 FAILURE_NONFINITE_OUTPUT=11
uint8 FAILURE_DEADLINE=12
uint8 FAILURE_RESET=13

uint64 trajectory_id
uint64 sequence
bool command_published
bool reference_valid
bool vehicle_state_valid
bool prediction_valid
float64 reference_age_seconds
float64 vehicle_state_age_seconds
float64 update_period_seconds
float64 solve_time_seconds
int32[3] solver_iterations
uint32 prediction_size
uint32 constraint_violation_count
uint32 consecutive_failures
bool deadline_missed
bool reset_detected
bool tracking_error_valid
float64[3] tracking_error_position
float64 tracking_error_norm
string detail
