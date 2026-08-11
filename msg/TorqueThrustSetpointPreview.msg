std_msgs/Header header
uint64 px4_timestamp
# Desired attitude in PX4 FRD/NED convention, diagnostic only.
float32[4] q_d_wxyz
# Commands sent to PX4's control allocator.
float32[3] torque_body_frd
float32[3] thrust_body_frd
# SO(3) feedback terms in body FLU.
float64[3] attitude_error
float64[3] body_rate_error_rad_s
bool torque_saturated
bool valid
bool prestream_enabled
bool px4_offboard_active
uint8 state
# SITL-first gates. Numeric validity below is diagnostic-only in this phase.
float64 state_age_seconds
bool state_fresh
float64 attitude_age_seconds
float64 body_rate_age_seconds
bool attitude_fresh
bool body_rate_fresh
bool m3_fresh
bool timesync_fresh
bool yaw_hold_valid
float64 yaw_hold_enu_rad
bool vehicle_state_valid
bool m3_message_valid
bool m3_math_valid
# PX4 hover-thrust estimator evidence and the calibration actually used for
# force [N] -> VehicleThrustSetpoint normalized thrust conversion.
float32 hover_thrust_estimate
float32 hover_thrust_estimate_variance
float64 hover_thrust_estimate_age_seconds
bool hover_thrust_estimate_valid
bool hover_thrust_estimate_fresh
float64 active_hover_thrust_normalized
bool active_hover_thrust_from_estimator
# Freshness is measured at the attitude-adapter callback boundary. A NaN age
# means that the corresponding stream has not been received yet.
float64 m3_age_seconds
float64 timesync_age_seconds
# Producer-to-adapter receipt timing. These fields separate a delayed/lost M3
# DDS delivery from a missing M3 timer publication.
uint64 m3_sequence
uint64 m3_publisher_steady_timestamp_ns
uint64 m3_adapter_received_steady_timestamp_ns
float64 m3_delivery_delay_seconds
uint8 failure_reason

uint8 STATE_WAIT_DATA=0
uint8 STATE_PRESTREAM=1
uint8 STATE_ACTIVE=2
uint8 STATE_OFFBOARD_LOST=3
