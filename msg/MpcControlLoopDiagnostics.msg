std_msgs/Header header

# One record per M2/M3 wall-timer invocation, including rejected cycles.
uint64 timer_sequence
uint64 configured_period_ns
uint64 timer_expected_fire_steady_timestamp_ns
uint64 timer_actual_start_steady_timestamp_ns
uint64 timer_callback_end_steady_timestamp_ns
float64 scheduling_lateness_ms
float64 callback_duration_ms

# Receipt timestamps are from the M2 ROS callbacks, measured on steady_clock.
uint64 reference_callback_sequence
uint64 state_callback_sequence
uint64 reference_received_steady_timestamp_ns
uint64 state_received_steady_timestamp_ns
float64 reference_receipt_age_seconds
float64 state_receipt_age_seconds

# Publication timestamps make a missing M2/M3 message distinguishable from a
# late timer callback or from later DDS delivery loss.
uint64 m2_sequence
uint64 m3_sequence
uint64 m2_published_steady_timestamp_ns
uint64 m3_published_steady_timestamp_ns
bool m2_published
bool m3_published

uint8 STAGE_NOT_READY=1
uint8 STAGE_INPUT_STALE=2
uint8 STAGE_INPUT_INVALID=3
uint8 STAGE_ACCELERATION_REJECTED=4
uint8 STAGE_REFERENCE_REJECTED=5
uint8 STAGE_SOLVER_REJECTED=6
uint8 STAGE_M3_REJECTED=7
uint8 STAGE_M3_PUBLISHED=8
uint8 stage
