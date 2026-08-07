# mpc_control_px4

PX4 v1.17 no-arm adapter and setpoint publisher for the normalized MPC
trajectory command.

`Px4TrajectoryAdapter` performs exactly one conversion:

```text
ROS ENU map command -> PX4 local NED TrajectorySetpoint
```

It converts position, velocity and acceleration, maps yaw/yaw-rate when yaw
is valid, sets unknown jerk to NaN for logging-only semantics, and validates
finite output plus strictly increasing PX4 microsecond timestamps.

The adapter does not send actuator commands or arm PX4. The publisher is a
separate ROS 2 node that supplies the PX4-time timestamp, publishes
`TrajectorySetpoint` and publishes the `OffboardControlMode` heartbeat. It does
not create `VehicleCommand` or actuator publishers.

## M6-WP2 publisher policy

The executable is `px4_trajectory_setpoint_publisher`.

- Input `trajectory_command` uses reliable ROS QoS.
- PX4-facing `timesync_topic`, `trajectory_setpoint_topic` and
  `offboard_control_mode_topic` use best-effort QoS.
- `TrajectorySetpoint` is published at 50 Hz by default.
- `OffboardControlMode` is published independently at 10 Hz by default.
- The heartbeat selects `position=true`; all other Offboard control levels are
  false. Velocity and acceleration remain finite feedforward fields in the
  trajectory setpoint.
- The publisher accepts only DDS timesync status with a nonzero PX4 timestamp.
  It interpolates between the latest PX4 timestamp anchor and the local
  monotonic clock, then enforces strictly increasing output timestamps.
- The publisher accepts only fresh `fmu/out/vehicle_local_position_v1` samples
  with valid XY/Z position, XY/Z velocity, control-ready heading and finite
  position/velocity/heading fields.
- Any change in PX4 local-position reset counters latches output off. Recovery
  requires a valid estimator plus a command with a newer `trajectory_id` or
  `sequence`; the previous command is never replayed after an estimator reset.
- No setpoint or heartbeat is published when the command is older than
  `command_timeout_seconds` (default 0.25 s) or the timesync anchor is older
  than `timesync_timeout_seconds` (default 0.5 s).
- No arm, disarm, mode-change, actuator, failsafe override or replay behavior
  is implemented here.

The timestamp interpolation is a host-side bridge for this no-arm wrapper; its
live alignment with PX4 must still be verified against the real PX4 topic in
M6/M7 SITL.

Relevant parameters are `command_topic`, `timesync_topic`,
`estimator_topic`,
`trajectory_setpoint_topic`, `offboard_control_mode_topic`,
`input_frame_id`, `setpoint_rate_hz`, `heartbeat_rate_hz`,
`command_timeout_seconds`, `timesync_timeout_seconds` and
`estimator_timeout_seconds`.

Build after sourcing the pinned `px4_msgs` overlay:

```bash
source /opt/ros/jazzy/setup.bash
source install/ros_px4_msgs/setup.bash
colcon build --base-paths ros --packages-up-to mpc_control_px4 \
  --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths ros --packages-select mpc_control_px4
colcon test-result --verbose
```

The M6-WP2 topic-level echo test is run by the gtest executable. It verifies
ENU-to-NED fields, finite/NaN policy, independent heartbeat rate behavior,
strictly increasing timestamps, timeout stopping and absence of arm/actuator
publishers. It uses isolated test topic names and never contacts PX4.
