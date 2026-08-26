# Module & Executable Registry (MODULE_REGISTRY.md)

This document provides a comprehensive inventory of every executable, ROS 2 node, library, and domain component in the repository.

---

## 1. ROS 2 Executables & Nodes

### 1.1 `vehicle_state_bridge_node`
* **File Location**: [`src/bridge/vehicle_state_bridge_node.cpp`](file:///home/ubuntu/Dev/mpc_controller/mpc_control/src/bridge/vehicle_state_bridge_node.cpp)
* **Purpose**: Ingests asynchronous PX4 state topics, validates freshness, evaluates cross-stream sample skew, performs Hamilton quaternion transformations from NED/FRD to ENU/FLU, and publishes a synchronized `VehicleState`.
* **Subscribed Topics**:
  - `/fmu/out/vehicle_local_position` (`px4_msgs::msg::VehicleLocalPosition`)
  - `/fmu/out/vehicle_attitude` (`px4_msgs::msg::VehicleAttitude`)
  - `/fmu/out/vehicle_angular_velocity` (`px4_msgs::msg::VehicleAngularVelocity`)
* **Published Topics**:
  - `/vehicle_state_bridge_node/vehicle_state` (`mpc_controller::msg::VehicleState`)
* **Active Parameters**:
  - `state_timeout_seconds` (default: $0.25\text{ s}$)
  - `max_sample_skew_seconds` (default: $0.10\text{ s}$)
  - `publish_rate_hz` (default: $50.0\text{ Hz}$)
* **Failure Modes & Safety**: If position, attitude, or rate streams exceed timeout ($50\text{ ms}$) or mutual timestamp skew exceeds $35\text{ ms}$, publication is suppressed and warning reasons (`POSITION_STALE`, `ATTITUDE_STALE`, `SAMPLE_SKEW`) are logged.

---

### 1.2 `reference_generator_node`
* **File Location**: [`src/mission/reference_generator_node.cpp`](file:///home/ubuntu/Dev/mpc_controller/mpc_control/src/mission/reference_generator_node.cpp)
* **Purpose**: Manages mission lifecycle (Parse $\to$ Arm $\to$ Takeoff $\to$ Hold $\to$ Waypoint Traverse $\to$ Landing), generates $C^2$ quintic polynomial splines with 3D bisector velocity smoothing at corners, and publishes 10-step physical reference preview.
* **Subscribed Topics**:
  - `/vehicle_state_bridge_node/vehicle_state` (`mpc_controller::msg::VehicleState`)
  - `/fmu/out/vehicle_status` (`px4_msgs::msg::VehicleStatus`)
* **Published Topics**:
  - `/reference_generator_node/reference_trajectory` (`mpc_controller::msg::ReferenceTrajectory`)
  - `/reference_generator_node/external_mode_active` (`std_msgs::msg::Bool`)
  - `/reference_generator_node/external_mode_hold` (`mpc_controller::msg::VehicleState`)
  - `/reference_generator_node/trajectory_visualization` (`visualization_msgs::msg::MarkerArray`)
* **Services**:
  - `/reference_generator_node/start_mission` (`std_srvs::srv::Trigger`)
  - `/reference_generator_node/reset_mission` (`std_srvs::srv::Trigger`)
* **Active Parameters**:
  - `mission_file_path` (string, path to JSON)
  - `mission_acceptance_radius_m` (default: $2.5\text{ m}$)
  - `mission_speed_override_m_s` (default: $0.0\text{ m/s}$)
  - `frame_id` (default: `"map"`)
  - `state_topic` (default: `"vehicle_state"`)
  - `auto_capture_current_hold` (default: `true`)
  - `hold_yaw_rad` (default: `0.0\text{ rad}`)
  - `horizon_seconds` (default: $30.0\text{ s}$)
  - `sample_period_seconds` (default: $0.1\text{ s}$)
  - `publish_rate_hz` (default: $50.0\text{ Hz}$)
  - `visualization_enabled` (default: `true`)
  - `visualization_publish_rate_hz` (default: $20.0\text{ Hz}$)

---

### 1.3 `mpc_controller_node`
* **File Location**: [`src/controller/mpc_controller_node.cpp`](file:///home/ubuntu/Dev/mpc_controller/mpc_control/src/controller/mpc_controller_node.cpp)
* **Purpose**: The main control loop node ($50\text{ Hz}$, $20\text{ ms}$). Ingests state and reference, converts physical references to TMPC state/control space, executes SQP-RTI via `AcadosTpmcSolver`, applies `command_safety_limiter`, and activates geometric PD fallback upon failure.
* **Subscribed Topics**:
  - `/vehicle_state_bridge_node/vehicle_state` (`mpc_controller::msg::VehicleState`)
  - `/reference_generator_node/reference_trajectory` (`mpc_controller::msg::ReferenceTrajectory`)
  - `/reference_generator_node/external_mode_active` (`std_msgs::msg::Bool`)
  - `/reference_generator_node/external_mode_hold` (`mpc_controller::msg::VehicleState`)
  - `/px4_attitude_mode_node/hover_thrust` (`std_msgs::msg::Float64`)
* **Published Topics**:
  - `/mpc_controller_node/force_attitude_setpoint` (`mpc_controller::msg::ForceAttitudeSetpoint`)
  - `/mpc_controller_node/translational_output` (`mpc_controller::msg::MpcTranslationalOutput`)
* **Active Parameters**:
  - `update_rate_hz` ($50.0\text{ Hz}$)
  - `strict_validation` (`true`)
  - `reference_timeout_seconds` ($1.5\text{ s}$)
  - `state_timeout_seconds` ($0.25\text{ s}$)
  - `output_frame_id` (`"map"`)
  - `sample_time_seconds` ($0.05\text{ s}$)
  - `solver_deadline_seconds` ($0.018\text{ s}$)
  - `gravity_m_s2` ($9.80665\text{ m/s}^2$)
  - `roll_time_constant_seconds`, `pitch_time_constant_seconds` ($0.18\text{ s}$)
  - `yaw_natural_frequency_rad_s` ($3.42\text{ rad/s}$), `yaw_damping_ratio` ($0.102$)
  - `collective_time_constant_seconds` ($0.0932\text{ s}$)
  - `stage_weights`, `terminal_weights`, `input_weights`
  - `yaw_command_delta_weight` ($25.0$)
  - `min_collective_specific_force_m_s2` ($7.0\text{ m/s}^2$), `max_collective_specific_force_m_s2` ($14.0\text{ m/s}^2$)
  - `collective_handover_valid_samples` ($5$), `handover_minimum_duration_seconds` ($1.0\text{ s}$), `handover_maximum_yaw_rate_rad_s` ($0.15\text{ rad/s}$)
  - `collective_measurement_filter_time_constant_seconds` ($0.15\text{ s}$)
  - `max_tilt` ($0.7854\text{ rad} = 45^\circ$), `max_tilt_rate_rad_s` ($2.0\text{ rad/s}$)
  - `max_yaw_command_rad` ($10^6\text{ rad}$), `max_yaw_command_rate_rad_s` ($2.0\text{ rad/s}$), `max_yaw_rate_rad_s` ($2.0\text{ rad/s}$)
  - `max_collective_rate_m_s3` ($25.0\text{ m/s}^3$)
  - `command_filter_alpha` ($0.50$)
  - `recovery_velocity_gain` ($1.0$), `recovery_position_gain` ($0.5$)
  - `recovery_max_acceleration_xy` ($2.5\text{ m/s}^2$), `recovery_max_acceleration_z` ($1.5\text{ m/s}^2$)

---

### 1.4 `px4_attitude_mode_node`
* **File Location**: [`src/controller/px4_attitude_mode_node.cpp`](file:///home/ubuntu/Dev/mpc_controller/mpc_control/src/controller/px4_attitude_mode_node.cpp)
* **Purpose**: Implements native PX4 External Mode via `px4_ros2_cpp`. Orchestrates autonomous arming/takeoff sequence through `MpcModeExecutor`, maps specific force $a$ to normalized thrust $T \in [-1, 0]$, and streams attitude setpoints to PX4.
* **Subscribed Topics**:
  - `/mpc_controller_node/force_attitude_setpoint` (`mpc_controller::msg::ForceAttitudeSetpoint`)
  - `/fmu/out/hover_thrust_estimate` (`px4_msgs::msg::HoverThrustEstimate`)
  - `/fmu/out/vehicle_status` (`px4_msgs::msg::VehicleStatus`)
* **Services**:
  - `/mpc_mode_executor/start` (`std_srvs::srv::Trigger`)
* **Active Parameters**:
  - `px4_system_id` (default: `2`)
  - `mission_timeout_seconds` (default: $300.0\text{ s}$)
  - `command_safety_max_tilt_rad` ($0.7854\text{ rad} = 45^\circ$)
  - `command_safety_min_collective_specific_force_m_s2` ($7.0\text{ m/s}^2$)
  - `command_safety_max_collective_specific_force_m_s2` ($14.0\text{ m/s}^2$)
  - `command_safety_max_collective_rate_m_s3` ($25.0\text{ m/s}^3$)
  - `command_safety_max_yaw_rate_rad_s` ($2.0\text{ rad/s}$)

---

### 1.5 `collective_identification_node`
* **File Location**: [`src/controller/collective_identification_node.cpp`](file:///home/ubuntu/Dev/mpc_controller/mpc_control/src/controller/collective_identification_node.cpp)
* **Purpose**: Generates sinusoidal vertical excitation trajectories during hover to record input-output pairs $(a_{cmd}, a_{meas})$ for identifying vertical specific force dynamics and hover thrust.
* **Launch File**: [`launch/collective_identification.launch.py`](file:///home/ubuntu/Dev/mpc_controller/mpc_control/launch/collective_identification.launch.py)
* **Active Parameters**: All 19 parameters defined in `config/controller.yaml` under `collective_identification_node`.

---

## 2. Core Libraries

* **`mpc_core`** (Static C++ Library):
  - `src/solver/tpmc_model.cpp`
  - `src/solver/tpmc_constraints.cpp`
  - `src/solver/tpmc_reference.cpp`
  - `src/solver/tpmc_solver.cpp`
  - `src/solver/acados_tpmc_solver.cpp`
* **`acados_tpmc_generated`** (Static C Library):
  - Compiled from auto-generated CasADi C sources in `build/acados_tpmc/`.
