# MPC Controller for PX4 Offboard

This ROS 2 package implements a measured-state MPC with geometric SO(3)
torque/thrust Offboard output:

```text
reference_generator_node -> mpc_controller_node -> px4_torque_thrust_setpoint_node -> PX4
               vehicle_state_bridge_node -------^
```

The translational MPC runs at 50 Hz and converts the reference into desired
specific force and attitude. A 250 Hz geometric SO(3) loop then publishes
torque and thrust to PX4. `HoverThrustEstimate` calibrates the thrust mapping;
`hover_thrust_normalized` is used only as a fallback, so vehicle mass is not a
controller parameter.

The torque loop reads PX4 attitude and angular velocity directly. The MPC uses
the validated `/vehicle_state`; its state bridge rejects invalid, stale,
out-of-order, or cross-topic-skewed samples.

The reusable controller code is kept under `include/mpc_controller`:

- `mpc_solver.hpp`: project API for one-axis MPC.
- `translational_mpc.hpp`: runs that API for the X, Y and Z axes.
- `force_attitude_mapping.hpp`: converts desired acceleration into specific
  force and attitude.
- `geometric_controller.hpp`: SO(3) torque control and PX4 thrust mapping.
- `state_bridge.hpp`: PX4 frame conversion and state timing checks.
- `reference_model.hpp`: hold/line/circle models and the rolling joystick planner.

OSQP 1.0.0 is the only QP solver. CMake downloads the pinned version. The local
`mpc_solver.hpp`/`mpc_solver.cpp` pair is only a thin wrapper: it converts the
MPC model and bounds to an OSQP problem, calls OSQP, and returns a small result
object. This keeps OSQP types out of the rest of the controller. There is no
second custom solver or hand-written ADMM loop.

Node-specific checks stay inside their node source. In particular, the strict
Z-acceleration gate is local to `mpc_controller_node.cpp`; it is not a reusable
controller API.

Successive final desired rotations are differentiated at the 50 Hz
force/attitude-setpoint boundary

The acceleration prediction uses the identified first-order time constants
from `model_time_constant_xyz`. The supplied x500 profile uses
`tau_xy=0.11 s` and `tau_z=0.13 s`; another airframe must identify only these
response constants rather than changing controller code. Each
horizon interval computes `alpha(dt)=exp(-dt/tau)`.

The first prediction interval is `dt_first=0.01 s`; later intervals use
`dt_later = 0.2s.

## Build only

```bash
make build
source install/setup.bash
```

Use `setup.bash`, not `local_setup.bash`: the former also sources the recorded
`px4_msgs` underlay required by ROS CLI tools to decode PX4 topics.

## Simulation and Offboard workflow

Start the complete simulator stack from the repository root:

```bash
make sim
```

`make sim` checks the environment, builds the package with one parallel worker,
then starts PX4 SITL.

```bash
make status
```

Use `make logs` if one of the processes did not start.

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

Ensure QGroundControl is not holding the FC USB serial port.
`make hil-check` reports an error when another process actually owns the port:

```bash
make hil-check
make hil             # equivalent to: make jmavsim
make status
```

`make hil` finds the PX4 `/dev/serial/by-id` device, starts jMAVSim at 921600
baud and 250 Hz, and writes its output to `/tmp/mpc_controller_sim/jmavsim_hil.log`.
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
- `/force_attitude_setpoint` -> `/fmu/in/vehicle_torque_setpoint`
- `/force_attitude_setpoint` -> `/fmu/in/vehicle_thrust_setpoint`
- `/fmu/out/hover_thrust_estimate` supplies hover-thrust calibration
- `/fmu/out/control_allocator_status` reports achieved wrench and actuator saturation


PX4 publishes this diagnostic with transient-local durability. Inspect it with:

```bash
source install/setup.bash
ros2 topic echo /fmu/out/control_allocator_status --once \
  --qos-reliability best_effort --qos-durability transient_local

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
quintic ramp and period from `circle_acceleration_limit_m_s2`. Keep the MPC
speed limit above this reference cap so the optimum is not pinned to a bound.
