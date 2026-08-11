# MPC Controller for PX4 Offboard

This ROS 2 package implements the active attitude-setpoint Offboard pipeline:

```text
reference_generator_node -> mpc_controller_node -> px4_torque_thrust_setpoint_node -> PX4
               vehicle_state_bridge_node -------^
```

The translational MPC runs at 50 Hz and the direct torque loop at 250 Hz. The
pipeline captures the current position and yaw before Offboard, solves the
independent-axis translational MPC, constructs the desired
force and attitude, closes a normalized SO(3) attitude/rate feedback loop, and
publishes torque plus collective thrust directly to PX4's control allocator. A
fresh valid `HoverThrustEstimate` initializes the Offboard thrust mapping and
is then followed through a rate-limited low-pass filter; the configured hover
thrust is only a fallback. Brief invalid estimator samples do not overwrite the
most recent valid estimate; that sample remains usable only for the configured
HTE timeout. The Gazebo x500 airframe uses PX4's native
`THR_MDL_FAC` compensation so the ROS force-to-thrust mapping remains linear.
Its fallback hover thrust is therefore `0.60`, close to the compensated HTE,
rather than the uncompensated value used by the earlier actuator profile.

The torque loop reads PX4 attitude and angular velocity directly with sensor
QoS. The aggregated `/vehicle_state` remains the measured input of the slower
translational MPC and is not the inner-loop feedback path.

The Z prediction model uses the identified first-order time constant from
`model_time_constant_xyz`. With `tau_z=0.13 s`, each horizon interval computes
`alpha(dt)=exp(-dt/tau_z)`; XY retains the direct acceleration-command model.
The optimizer input `u` is sent to the thrust path, while `a[k+1]` remains the
predicted delayed plant response published for diagnostics.

## Build only

```bash
make build
source install/local_setup.bash
```

Use this only when a standalone package build is needed. The normal simulation
workflow below runs the build automatically. Only the four runtime nodes and
their message interfaces are built.

## Simulation and Offboard workflow

Start the complete simulator stack from the repository root:

```bash
make sim
```

`make sim` checks the environment, builds the package with one parallel worker,
then starts PX4 SITL, Gazebo, the uXRCE-DDS agent, and exactly one MPC ROS 2
pipeline. Wait for Gazebo and PX4 to finish starting, then check the processes:

```bash
make status
```

Use `make logs` if one of the processes did not start. Do not run an additional
`ros2 launch mpc_controller mpc_offboard.launch.py` after `make sim`; a second
pipeline would publish duplicate references and setpoints.

In a second terminal, source the generated ROS workspace and verify that state
data is available:

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 topic echo /vehicle_state --once
```

The operator then performs the flight-mode sequence in PX4/QGroundControl:

1. Arm and take off in Position mode.
2. Establish a stable hover at the desired starting position.
3. Change to Offboard mode. The current position and yaw are captured at the
   Offboard rising edge.
4. Confirm that the adapter is active before sending a trajectory command:

```bash
ros2 topic echo /px4_torque_thrust_setpoint_preview --once
```

The expected fields are `px4_offboard_active: true`, `state: 2`, and all three
freshness fields set to `true`. Only then call `start_line`, `start_circle`, or
publish a relative step. The launch file never arms the vehicle and never
requests Offboard mode itself.

After the flight, change back to Position mode before stopping the simulator:

```bash
make stop
```

Runtime parameters are split between:

- `config/controller.yaml`: reference generator, state bridge, MPC, and force model.
- `config/offboard.yaml`: current simulation profile and PX4 attitude adapter.

The main command path is:

- `/reference_trajectory` -> `/mpc_translational_output`
- `/m3_control_output` -> `/fmu/in/vehicle_torque_setpoint`
- `/m3_control_output` -> `/fmu/in/vehicle_thrust_setpoint`
- `/fmu/out/hover_thrust_estimate` supplies hover-thrust calibration

Use `/reference_step` for relative point commands while testing.

For the line test, enter Offboard on the captured hold first, then start the
configured relative line exactly once:

```bash
ros2 service call /reference_generator_node/start_line std_srvs/srv/Trigger "{}"
```

The default validation profile uses a 2 m ENU X line over 12 s while holding
the captured Z and yaw. The line phase advances continuously across the 1 Hz
reference publications, and the endpoint is held after completion.

For the circle test, enter Offboard on a newly captured hold and start the
configured circle exactly once:

```bash
ros2 service call /reference_generator_node/start_circle std_srvs/srv/Trigger "{}"
```

Set only `circle_radius` for the geometry. The generator bounds cruise speed by
`circle_reference_speed_limit_m_s` and centripetal acceleration, then derives a
quintic ramp and period from `circle_acceleration_limit_m_s2`. The configured
4 m/s reference cap leaves tracking margin below 5 m/s measured speed.
The circle begins at the captured position without a setpoint jump, initially
moves in ENU +Y for `circle_direction: 1`, and holds the captured Z and yaw.
After one revolution it stops and holds the start point. Leaving Offboard
returns the generator to current-state hold tracking and cancels the active
line or circle.
