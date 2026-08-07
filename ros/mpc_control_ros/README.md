# mpc_control_ros

ROS 2 Jazzy lifecycle wrapper for the standalone MPC virtual trajectory core.

This package owns only ROS concerns:

- lifecycle transitions;
- reference and measured-state subscriptions;
- reference/state validation and timeout handling;
- normalized command, prediction and diagnostics publication.

It intentionally does not depend on `px4_msgs`, Gazebo or PX4. Frame
conversion to PX4 NED belongs to the later M6 adapter.

## Topics

Parameters control the topic names. Defaults are:

```text
reference_trajectory  mpc_control_msgs/msg/ReferenceTrajectory
vehicle_state         mpc_control_msgs/msg/VehicleState
trajectory_command    mpc_control_msgs/msg/TrajectoryCommand
predicted_trajectory  mpc_control_msgs/msg/PredictedTrajectory
mpc_diagnostics       mpc_control_msgs/msg/MpcDiagnostics
```

The command and prediction publishers are lifecycle publishers. They are
inactive until the node is in the `ACTIVE` state and are never published for
invalid, stale or non-finite inputs. Diagnostics uses a normal publisher so
inactive/error evidence remains observable.

The wrapper also fails closed on ROS-clock anomalies. A backward clock jump,
a forward discontinuity larger than `clock_jump_threshold_seconds`, or a
clock that remains unchanged longer than `clock_stall_timeout_seconds` stops
valid command publication and reports `FAILURE_TIME_JUMP`.

## Integration tests

The package test executable covers the M5-WP2 ROS boundary cases:

- simulated ROS clock moving backwards: command publication stops and
  `FAILURE_TIME_JUMP` is reported;
- vehicle-state timestamp moving backwards: the stream stops and
  `FAILURE_OUT_OF_ORDER` is retained in diagnostics;
- best-effort input publishers against the wrapper's reliable subscriptions:
  no input callback is accepted and no command is published;
- node destruction and recreation on the same topics: a fresh active stream is
  created and its command sequence restarts at zero.

Run the wrapper test with:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --base-paths ros --packages-select mpc_control_ros \
  --event-handlers console_direct+
colcon test-result --verbose
```
