# MPC Controller for PX4 Offboard

This ROS 2 package implements a measured-state MPC with geometric SO(3)
torque/thrust Offboard output:

```text
reference_generator_node -> mpc_controller_node -> px4_torque_thrust_setpoint_node -> PX4
               vehicle_state_bridge_node -------^
```

The translational MPC runs at 50 Hz and the direct torque loop at 250 Hz. The
pipeline captures the current position and yaw before Offboard, solves the
independent-axis translational MPC, constructs the desired mass-normalized
specific force and attitude, closes a normalized geometric SO(3) attitude/rate
feedback loop, and publishes torque plus collective thrust directly to PX4's
control allocator. A
fresh valid `HoverThrustEstimate` initializes the Offboard thrust mapping and
is then followed through a rate-limited low-pass filter; the configured hover
thrust is only a fallback. Brief invalid estimator samples do not overwrite the
most recent valid estimate; that sample remains usable only for the configured
HTE timeout. Vehicle mass is not a parameter: it cancels analytically because
the controller-to-PX4 contract is specific force in m/s^2. The only static
thrust calibration is `hover_thrust_normalized`, used solely when HTE is absent.
For each airframe, configure PX4's own thrust model and allow HTE to converge.

The torque loop reads PX4 attitude and angular velocity directly with sensor
QoS. The aggregated `/vehicle_state` remains the measured input of the slower
translational MPC and is not the inner-loop feedback path.

The state bridge rejects zero, reordered and cross-topic-skewed PX4 samples.
PX4 boot-time and synchronized Unix-epoch timestamps keep independent
monotonic histories because Gazebo can switch between those clock domains.
Backward jumps within either domain remain rejected as stale data.

The reusable controller code is kept under `include/mpc_controller`:

- `mpc_solver.hpp`: small OSQP-backed one-axis solver API.
- `translational_mpc.hpp`: combines the X/Y/Z solvers and samples references.
- `force_attitude_mapping.hpp`: converts desired acceleration into specific
  force and attitude.
- `geometric_controller.hpp`: SO(3) torque control and PX4 thrust mapping.
- `state_bridge.hpp`: PX4 frame conversion and state timing checks.
- `reference_model.hpp`: hold/line/circle models and the rolling joystick planner.

The active solver is OSQP 1.0.0, pinned and built by CMake. The package keeps
the MPC formulation local but delegates factorization, ADMM iteration,
warm-starting, infeasibility detection, residuals, and the solve time limit to
OSQP. The former hand-written ADMM loop and the inactive
`mrs_mpc_solvers`/CVXGEN submodule have been removed.

Node-specific checks stay inside their node source. In particular, the strict
Z-acceleration gate is local to `mpc_controller_node.cpp`; it is not a reusable
controller API.

Successive final desired rotations are differentiated at the 50 Hz
force/attitude-setpoint boundary
to obtain bounded desired body rate and angular acceleration. The rate error is
`omega - R^T R_d omega_d`; this replaces the earlier assumption that desired
body rate is always zero. Rigid-body feedforward is implemented but remains
disabled until `normalized_inertia = J / torque_scale` has been identified for
the active UAV, because PX4 `VehicleTorqueSetpoint` uses normalized units rather
than N*m. The active state is visible in
`/px4_torque_thrust_setpoint_preview`.

The acceleration prediction uses the identified first-order time constants
from `model_time_constant_xyz`. The supplied x500 profile uses
`tau_xy=0.11 s` and `tau_z=0.13 s`; another airframe must identify only these
response constants rather than changing controller code. Each
horizon interval computes `alpha(dt)=exp(-dt/tau)`. This is a compact
approximation of the measured response delay, not an exact transport-delay
state. The optimizer input `u` is sent to the thrust path, while `a[k+1]`
remains the predicted delayed plant response published for diagnostics.

The first shooting interval is the independent `dt_first=0.01 s`; it remains
shorter than the 20 ms controller period. Before each solve, the warm start
advances the expired first command while preserving the longer prediction
knots. OSQP manages its ADMM penalty and convergence checks. A whole-solve
deadline of 18 ms prevents the optimizer from blocking the 50 Hz callback.

Velocity and acceleration use a recovery envelope: nominal bounds remain hard
in normal flight, but if the measured/model state is already outside a bound,
the envelope admits a rate-limited command that does not worsen that violation.
If a solve still fails or misses its deadline, the controller immediately
publishes bounded velocity braking, marks the MPC output invalid, and the
reference generator cancels an active line or circle to current-position hold.

## Build only

```bash
make build
source install/setup.bash
```

Use `setup.bash`, not `local_setup.bash`: the former also sources the recorded
`px4_msgs` underlay required by ROS CLI tools to decode PX4 topics.

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
`setpoint_fresh`, and `timesync_fresh` set to `true`. The default
`reference_input_mode` is `manual_velocity`. Centered sticks hold the captured
position; stick motion is converted into a rolling, jerk-limited trajectory by
the predictive shared-control planner described below. The launch file never
arms the vehicle and never requests Offboard mode itself.

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
- `/force_attitude_setpoint` -> `/fmu/in/vehicle_torque_setpoint`
- `/force_attitude_setpoint` -> `/fmu/in/vehicle_thrust_setpoint`
- `/fmu/out/hover_thrust_estimate` supplies hover-thrust calibration
- `/fmu/out/control_allocator_status` reports achieved wrench and actuator saturation

The controller intentionally separates portable values from airframe tuning:

- Supplied by PX4 at runtime: attitude, body rate, HTE thrust calibration, and
  allocator feasibility/saturation.
- Derived internally: gravity compensation and specific-force-to-normalized-
  thrust scaling.
- Tuned per airframe: MPC weights, acceleration-response time constants, SO(3)
  gains, speed/acceleration/tilt/torque limits, and the HTE fallback only if the
  estimator is unavailable.

PX4 currently does not expose its velocity/tilt parameters through this DDS
profile, and it does not estimate normalized torque inertia. Those values stay
explicit in YAML instead of being silently duplicated as C++ constants.
The added allocator-status output changes PX4's DDS topic table, so rebuild and
restart PX4 once before expecting that diagnostic topic; its absence does not
block the command path.

PX4 publishes this diagnostic with transient-local durability. Inspect it with:

```bash
source install/setup.bash
ros2 topic echo /fmu/out/control_allocator_status --once \
  --qos-reliability best_effort --qos-durability transient_local
```

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

## Predictive shared-control planner

The manual path now optimizes reference continuously before the tracking MPC:

```text
joystick intent + measured state
              -> rolling candidate trajectories
              -> dynamic feasibility and collision-check interface
              -> lowest-cost ReferenceTrajectory
              -> tracking MPC -> geometric SO(3) -> torque/thrust
```

At each update the planner generates heading and speed motion primitives over
the configured 5 s horizon. Every candidate is acceleration- and jerk-limited,
then scored for joystick intent, progress, acceleration, jerk and branch
switching. Hysteresis prevents left/right branch chatter. A bounded braking
trajectory is always included; centered or stale sticks select braking and end
in position hold.

The supplied manual profile allows 2.5 m/s^2 horizontal acceleration and 4 m/s^3
horizontal jerk while retaining the 45-degree tilt ceiling. Pure vertical stick
input generates only one heading primitive per speed scale because rotated XY
headings would otherwise be duplicates.

This stage deliberately has no obstacle input. The collision predicate accepts
all dynamically feasible candidates, so without obstacles the selected red
reference should stay close to the green joystick intent. The predicate is an
explicit integration point for later hard collision rejection; obstacle
penalties are not being presented as a safety guarantee.

The relevant settings are `planner_heading_offsets_deg`,
`planner_speed_scales`, the `planner_*_weight` values, and `planner_hysteresis`.
The physical limits remain the `manual_max_*` speed, acceleration and jerk
parameters. In RViz, `/mpc_visualization` contains:

- green arrow: raw joystick direction in ENU;
- red arrow and red line: selected `ReferenceTrajectory`;
- cyan line: tracking MPC `predicted_states`.

After building and restarting, validate in this order: enter Offboard with
centered sticks, hold for 30 s, apply about 10% X stick, release to braking,
then repeat for Y, Z and yaw. Confirm that the red path follows the green intent,
the cyan prediction follows the red path, and releasing the stick returns to a
bounded hold before increasing stick amplitude.

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
The circle begins at the captured position without a setpoint jump, initially
moves in ENU +Y for `circle_direction: 1`, and holds the captured Z and yaw.
After one revolution it stops and holds the start point. Leaving Offboard
returns the generator to current-state hold tracking and cancels the active
line or circle.

Use the 5 m/s profile as the regression gate. Increase the reference cap only
after a complete bag has zero invalid MPC samples, zero deadline misses and no
allocator saturation: test 8 m/s, then 10 m/s, then 12 m/s as separate runs.
