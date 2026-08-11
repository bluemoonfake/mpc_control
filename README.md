# MPC Controller for PX4 Offboard

This ROS 2 package implements the active attitude-setpoint Offboard pipeline:

```text
reference_generator_node -> mpc_controller_node -> px4_attitude_setpoint_node -> PX4
               vehicle_state_bridge_node -------^                    
```

The controller runs at 50 Hz. It captures the current position and yaw before
Offboard, solves the independent-axis translational MPC, constructs the desired
force and attitude, and converts collective force to PX4 normalized thrust. A
fresh valid `HoverThrustEstimate` is latched when Offboard becomes active; the
configured hover thrust is only a fallback.

## Build

```bash
make build
source install/local_setup.bash
```

Only the four runtime nodes and their message interfaces are built. Historical
direct-wrench bridges, milestone launch files, timing probes, and standalone
test executables are not part of the deployable package.

## Run

```bash
ros2 launch mpc_controller mpc_offboard.launch.py
```

The launch file does not arm the vehicle and does not request Offboard mode.
The operator remains responsible for arming and changing mode in PX4.

Runtime parameters are split between:

- `config/controller.yaml`: reference generator, state bridge, MPC, and force model.
- `config/offboard.yaml`: current simulation profile and PX4 attitude adapter.

The main command path is:

- `/reference_trajectory` -> `/mpc_translational_output`
- `/m3_control_output` -> `/fmu/in/vehicle_attitude_setpoint`
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

The default one-revolution circle has radius 3 m and period 20 s, with 3 s quintic
acceleration and deceleration ramps. It begins at the captured
position without a setpoint jump, initially moves in ENU +Y, and holds the
captured Z and yaw. After one revolution it stops and holds the start point.
Leaving Offboard returns the generator to current-state hold tracking.
