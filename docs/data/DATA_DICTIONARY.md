# Data Dictionary & Telemetry Specifications (DATA_DICTIONARY.md)

This document specifies every field recorded in `/tmp/mpc_controller_sim/tpmc_metrics.csv` by [`scripts/recording/record_tpmc_metrics.py`](file:///home/ubuntu/Dev/mpc_controller/mpc_control/scripts/recording/record_tpmc_metrics.py).

---

## 1. Metric Field Definitions & Coordinate Contracts

| Column Header | Source Topic | Data Type | Units | Frame | Description | Used By Analysis Scripts |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `timestamp_s` | ROS Clock | `float64` | $\text{s}$ | — | ROS monotonic epoch time of recording. | All analysis scripts |
| `px`, `py`, `pz` | `/mpc_controller_node/translational_output` | `float64` | $\text{m}$ | ENU | Current vehicle position $(x, y, z)$. | Tracking RMSE, Clearance |
| `ref_px`, `ref_py`, `ref_pz` | `/mpc_controller_node/translational_output` | `float64` | $\text{m}$ | ENU | Target reference position $(x_{ref}, y_{ref}, z_{ref})$. | Tracking error calculation |
| `vx`, `vy`, `vz` | `/mpc_controller_node/translational_output` | `float64` | $\text{m/s}$ | ENU | Current vehicle translational velocity. | Velocity RMSE |
| `ref_vx`, `ref_vy`, `ref_vz`| `/mpc_controller_node/translational_output` | `float64` | $\text{m/s}$ | ENU | Target reference velocity. | Velocity error calculation |
| `roll_deg`, `pitch_deg`, `yaw_deg` | `/mpc_controller_node/translational_output` | `float64` | $\text{deg}$ | ENU/FLU | Vehicle Euler attitude angles. | Hover stability, Yaw error |
| `ref_yaw_deg` | `/mpc_controller_node/translational_output` | `float64` | $\text{deg}$ | ENU | Target yaw angle. | Heading error evaluation |
| `tilt_deg` | `/mpc_controller_node/translational_output` | `float64` | $\text{deg}$ | ENU | Vehicle total tilt angle $\theta = \arccos(R_{2,2})$. | Maximum tilt gate |
| `a_mps2` | `/mpc_controller_node/translational_output` | `float64` | $\text{m/s}^2$ | Body Z | Internal specific force state $a$. | Force dynamics analysis |
| `cmd_roll_deg`, `cmd_pitch_deg` | `/mpc_controller_node/translational_output` | `float64` | $\text{deg}$ | ENU | MPC optimized roll/pitch commands $u_\phi, u_\theta$. | Command smoothness & effort |
| `cmd_yaw_deg` | `/mpc_controller_node/translational_output` | `float64` | $\text{deg}$ | ENU | MPC optimized yaw command $u_\psi$. | Heading control effort |
| `cmd_a_mps2` | `/mpc_controller_node/translational_output` | `float64` | $\text{m/s}^2$ | Body Z | MPC optimized specific force command $u_a$. | Thrust command effort |
| `solve_time_ms` | `/mpc_controller_node/translational_output` | `float64` | $\text{ms}$ | — | acados solve wall-clock duration. | Solve time p95/p99, Max |
| `e2e_time_ms` | `/mpc_controller_node/translational_output` | `float64` | $\text{ms}$ | — | Total end-to-end controller loop execution duration. | Worst-case timing gate |
| `qp_iterations` | `/mpc_controller_node/translational_output` | `int32` | count | — | HPIPM QP solver iteration count. | Solver convergence audit |
| `kkt_residual` | `/mpc_controller_node/translational_output` | `float64` | — | — | acados maximum KKT stationarity residual. | Numerical quality audit |
| `solver_status` | `/mpc_controller_node/translational_output` | `int32` | enum | — | 0=SUCCESS, 1=FALLBACK, 2=TIMEOUT. | No solver failure gate |
| `fallback_active` | `/mpc_controller_node/translational_output` | `int32` | bool (0/1) | — | Indicator if Geometric Fallback was triggered. | No QP fallback gate |
| `motor_available` | `/fmu/out/actuator_motors` | `int32` | bool (0/1) | — | 1 if motor telemetry was received. | Motor saturation gate |
| `motor_age_ms` | `/fmu/out/actuator_motors` | `float64` | $\text{ms}$ | — | Age of motor message relative to MPC loop. | Freshness verification |
| `motor_min`, `motor_max` | `/fmu/out/actuator_motors` | `float64` | $[0, 1]$ | — | Min/Max normalized PWM actuator motor outputs. | Motor saturation gate |
| `external_mode_active` | `/fmu/out/vehicle_status` | `int32` | bool (0/1) | — | 1 when vehicle is in PX4 External Mode. | Mission window indexing |
