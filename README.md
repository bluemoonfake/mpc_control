# MPC Controller for PX4 Offboard

This ROS 2 package implements a measured-state MPC with geometric SO(3)
torque/thrust Offboard output:

```text
reference_generator_node -> mpc_controller_node -> px4_torque_thrust_setpoint_node -> PX4
               vehicle_state_bridge_node -------^
```

The translational MPC runs at 50 Hz and the direct torque loop at 250 Hz. The
pipeline captures the current position and yaw before Offboard, solves the
independent-axis translational MPC, constructs the desired
force and attitude, closes a normalized geometric SO(3) attitude/rate feedback
loop, and
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

The reusable controller code is kept under `include/mpc_controller`:

- `mpc_solver.hpp`: one-axis condensed MPC solver.
- `translational_mpc.hpp`: combines the X/Y/Z solvers and samples references.
- `mrs_control_math.hpp`: converts desired acceleration into force and attitude.
- `geometric_controller.hpp`: SO(3) torque control and PX4 thrust mapping.
- `state_bridge.hpp`: PX4 frame conversion and state timing checks.
- `reference_model.hpp`: hold, line and circle reference models.

Node-specific checks stay inside their node source. In particular, the strict
Z-acceleration gate is local to `mpc_controller_node.cpp`; it is not a reusable
controller API.

Successive final desired rotations are differentiated at the 50 Hz M3 boundary
to obtain bounded desired body rate and angular acceleration. The rate error is
`omega - R^T R_d omega_d`; this replaces the earlier assumption that desired
body rate is always zero. Rigid-body feedforward is implemented but remains
disabled until `normalized_inertia = J / torque_scale` has been identified for
the active UAV, because PX4 `VehicleTorqueSetpoint` uses normalized units rather
than N*m. The active state is visible in
`/px4_torque_thrust_setpoint_preview`.

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

The expected fields are `state: 2`, with `attitude_fresh`, `body_rate_fresh`,
`m3_fresh`, and `timesync_fresh` set to `true`. The default `reference_input_mode` is
`manual_velocity`, so centered sticks hold the captured position and joystick
motion generates a rate- and acceleration-limited receding trajectory. The
launch file never arms the vehicle and never requests Offboard mode itself.

After the flight, change back to Position mode before stopping the simulator:

```bash
make stop
```

## jMAVSim hardware-in-the-loop workflow

The HIL workflow uses the real PX4 flight controller over USB. Prepare and
flash the firmware explicitly; `make hil` never changes or flashes FC firmware:

```bash
make hil-config     # enable Modules -> Simulation -> pwm_out_sim
make hil-firmware   # build px4_fmu-v6x_default
make hil-upload     # explicitly flash the connected FC
```

If enabling `pwm_out_sim` exceeds the board's FLASH capacity, disable an
unused HIL driver such as `Drivers -> Camera Capture` in `make hil-config`,
then build again. In QGroundControl select `Simulation -> HIL Quadcopter X`
(`SYS_AUTOSTART=1001`, `SYS_HITL=1`) after flashing.

Ensure QGroundControl is not holding the FC USB serial port, then run from this
repository. QGroundControl may remain open if it is connected only through UDP;
`make hil-check` reports an error when another process actually owns the port:

```bash
make hil-check
make hil             # equivalent to: make jmavsim
make status
```

`make hil` finds the PX4 `/dev/serial/by-id` device, starts jMAVSim at 921600
baud and 250 Hz, and writes its output to
`/tmp/mpc_controller_sim/jmavsim_hil.log`. Open QGroundControl only after
jMAVSim is running; QGroundControl then connects through the UDP bridge. Use
`make logs`, `make hil-stop`, or `make stop` for diagnostics and shutdown.

The FC serial port and jMAVSim settings can be overridden when needed:

```bash
make hil HIL_DEVICE=/dev/ttyACM1 HIL_BAUD=921600 HIL_RATE_HZ=250
```

Runtime parameters are split between:

- `config/controller.yaml`: reference generator, state bridge, MPC, and force model.
- `config/offboard.yaml`: PX4 torque/thrust adapter and geometric SO(3) gains.

The main command path is:

- `/fmu/out/manual_control_setpoint` -> `/reference_trajectory`
- `/reference_trajectory` -> `/mpc_translational_output`
- `/m3_control_output` -> `/fmu/in/vehicle_torque_setpoint`
- `/m3_control_output` -> `/fmu/in/vehicle_thrust_setpoint`
- `/fmu/out/hover_thrust_estimate` supplies hover-thrust calibration

In manual mode, PX4 `pitch` commands body-forward velocity, `roll` commands
body-right velocity, `throttle` commands ENU vertical velocity, and `yaw`
commands a bounded ENU yaw rate. PX4's clockwise-positive yaw stick is converted
to counter-clockwise-positive ENU. Centering the yaw stick holds the last yaw.
The translation command is rotated into ENU with measured yaw; stale input
decelerates to position hold and sets yaw rate to zero. The configured reference
lead is bounded to 2 m horizontally and 1 m vertically.

Ensure bit 1 (Offboard override) of PX4 `COM_RC_OVERRIDE` is disabled; otherwise
stick motion makes PX4 leave Offboard for Position mode. The default value `1`
enables override for automatic modes only and is compatible. Keep an explicit
mode switch available for pilot takeover.

To restore autonomous trajectory tests, set
`reference_input_mode: trajectory` in `config/controller.yaml`, rebuild and
restart the pipeline. Only in that mode use `/reference_step`, `start_line`, or
`start_circle`; these commands are rejected in `manual_velocity` mode.

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
