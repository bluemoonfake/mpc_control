std_msgs/Header header
# Desired attitude in PX4 FRD/NED convention, diagnostic only.
float32[4] q_d_wxyz
# Commands sent to PX4's control allocator.
float32[3] torque_body_frd
float32[3] thrust_body_frd
# SO(3) feedback terms in body FLU.
float64[3] attitude_error
float64[3] body_rate_error_rad_s
# Final desired SO(3) kinematics in body FLU. These are derived from the
# held-yaw desired rotations, not copied from world-frame yaw rate.
float64[3] desired_body_rate_flu_rad_s
float64[3] desired_body_acceleration_flu_rad_s2
float64[3] feedback_torque_body_flu_normalized
float64[3] dynamics_torque_body_flu_normalized
bool desired_kinematics_valid
bool torque_saturated
bool mpc_recovery_active
# Feedback reported by PX4 after control allocation.
bool allocator_status_fresh
bool torque_setpoint_achieved
bool thrust_setpoint_achieved
bool actuator_saturated
float32[3] unallocated_torque
float32[3] unallocated_thrust
bool valid
uint8 state
# Freshness gates used by the adapter.
bool attitude_fresh
bool body_rate_fresh
bool setpoint_fresh
bool timesync_fresh
bool yaw_hold_valid
float64 yaw_hold_enu_rad
# PX4 HTE value and the calibration used for force-to-thrust conversion.
float32 hover_thrust_estimate
float64 active_hover_thrust_normalized
bool active_hover_thrust_from_estimator
uint8 failure_reason

uint8 STATE_WAIT_DATA=0
uint8 STATE_PRESTREAM=1
uint8 STATE_ACTIVE=2
uint8 STATE_OFFBOARD_LOST=3
