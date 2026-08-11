std_msgs/Header header
uint64 publisher_steady_timestamp_ns
uint64 timer_sequence
uint64 timer_expected_fire_steady_timestamp_ns
uint64 timer_actual_start_steady_timestamp_ns
uint64 timer_callback_end_steady_timestamp_ns

uint8 WAITING_FOR_READINESS=0
uint8 READY_NOT_STABLE=1
uint8 STABLE_DWELL=2
uint8 CAPTURED=3

uint8 state
bool vehicle_state_valid
bool vehicle_state_fresh
bool heading_valid
bool yaw_valid
bool control_ready
# Strict ENTER-band stability used to authorize explicit capture.
bool stable_hover
# Current-state capture authorization. This never triggers capture itself.
bool capture_ready
# Stateful pre-handover stability. It tolerates a short ENTER-band excursion
# but does not replace controller-health or envelope checks.
bool handover_stability_valid
bool dwell_complete
bool capture_valid
float64 stable_duration_seconds

# [v_xy, v_z, a_xy, a_z, omega_xy, omega_z, abs(roll), abs(pitch)]
float64[8] stability_metrics
# Same ordering as stability_metrics. Limits are copied from the configured
# stable-hover gate so downstream diagnostics can compare values without
# reconstructing parameters from another node.
float64[8] stability_limits

uint64 capture_timestamp
