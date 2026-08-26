# TMPC Controller for PX4

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot%20v1.14+-red.svg)](https://px4.io/)
[![acados](https://img.shields.io/badge/acados-SQP--RTI%20%2F%20HPIPM-green.svg)](https://docs.acados.org/)
[![C++17](https://img.shields.io/badge/standard-C%2B%2B17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Validation](https://img.shields.io/badge/SITL%20Gates-11%2F11%20ALL%20PASS-brightgreen.svg)](docs/validation/PASS_FAIL_CRITERIA.md)

A high-performance, real-time **Tracking Model Predictive Control (TMPC)** framework for multirotor UAVs integrated with **PX4 Autopilot** and **ROS 2**. The system leverages an embedded **acados** C solver executing real-time **SQP-RTI (Real-Time Iteration)** with **HPIPM** quadratic programming to achieve sub-millisecond solve latencies at a $50\text{ Hz}$ control rate.

---

## 1. System Architecture

The architecture adheres strictly to **Clean Architecture** principles, decoupling the mathematical domain and optimization solver from the ROS 2 and PX4 communication infrastructure:

```mermaid
graph TD
    subgraph PX4_Ecosystem["PX4 Autopilot & Simulation (SITL / FMU v6x)"]
        PX4_Sensors["Sensors / EKF2 State<br/>(Local Pos, Attitude, AngVel)"]
        PX4_Mixer["PX4 Attitude & Rate Controller<br/>+ Motor Mixer"]
        PX4_Mode["PX4 Mode Manager<br/>(External Mode Interface)"]
    end

    subgraph Bridge_Layer["State Bridge Subsystem"]
        StateBridge["vehicle_state_bridge_node<br/>- Ingests NED/FRD PX4 Topics<br/>- Skew & Freshness Gate (<35ms)<br/>- Converts to ENU/FLU"]
    end

    subgraph Mission_Layer["Mission & Trajectory Subsystem"]
        MissionParser["Mission Parser<br/>(JSON / Plan parser)"]
        TrajGen["minimum_time_trajectory<br/>- Quintic Polynomial Splines<br/>- 3D Corner Bisector Scaling"]
        RefNode["reference_generator_node<br/>- Streams ReferenceTrajectory<br/>- Manages Hold / Waypoint Advance"]
    end

    subgraph Controller_Layer["TMPC Controller Subsystem"]
        MPCNode["mpc_controller_node (50 Hz Timer)<br/>- Handover Hold Gate (5 samples)<br/>- Reference Conversion Engine<br/>- Fallback Geometric Controller"]
        SafetyLimiter["command_safety_limiter<br/>- Tilt Cone Projection<br/>- Slew-rate Limits"]
        
        subgraph Domain_Solver["TMPC Domain & Solver Engine"]
            TPMCModel["tpmc_model (RK4 Integrator)"]
            TPMCConstraints["tpmc_constraints (BGH Box & Tilt)"]
            AcadosSolver["acados_tpmc_solver (C++ Adapter)<br/>- RAII Capsule Manager<br/>- Shifted Warm Start"]
            GeneratedC["Generated acados C-Code<br/>(SQP-RTI, HPIPM QP Solver)"]
        end
    end

    subgraph Actuation_Layer["PX4 Mode & Actuation Subsystem"]
        AttitudeNode["px4_attitude_mode_node<br/>- MpcModeExecutor (Auto Sequence)<br/>- Force-to-Thrust Mapping<br/>- Attitude Setpoint Streaming"]
    end

    %% Signal Flows
    PX4_Sensors -->|uXRCE-DDS (NED/FRD)| StateBridge
    StateBridge -->|VehicleState (ENU/FLU)| RefNode
    StateBridge -->|VehicleState (ENU/FLU)| MPCNode
    
    MissionParser --> TrajGen
    TrajGen --> RefNode
    RefNode -->|ReferenceTrajectory| MPCNode
    
    MPCNode --> TPMCModel
    MPCNode --> TPMCConstraints
    MPCNode --> AcadosSolver
    AcadosSolver --> GeneratedC
    GeneratedC --> AcadosSolver
    AcadosSolver --> MPCNode
    MPCNode --> SafetyLimiter
    SafetyLimiter -->|ForceAttitudeSetpoint| AttitudeNode
    
    AttitudeNode -->|Attitude & Thrust Setpoint| PX4_Mixer
    PX4_Mode <-->|Mode Handshake| AttitudeNode
```

---

## 2. Mathematical Formulation

### 2.1 State and Control Vectors

The optimization problem tracks a $15$-dimensional augmented state vector $x \in \mathbb{R}^{15}$ and a $4$-dimensional control vector $u \in \mathbb{R}^4$:

$$
x = \big[ \underbrace{p_x, p_y, p_z}_{\text{position (ENU)}}, \underbrace{v_x, v_y, v_z}_{\text{velocity}}, \underbrace{\phi, \theta, \psi}_{\text{Euler angles}}, \underbrace{\dot{\psi}}_{\text{yaw rate}}, \underbrace{a}_{\text{specific force}}, \underbrace{u_{\phi,prev}, u_{\theta,prev}, u_{\psi,prev}, u_{a,prev}}_{\text{command memory for slew-rate limits}} \big]^T
$$

$$
u = [u_\phi, u_\theta, u_\psi, u_a]^T = [\text{roll command}, \text{pitch command}, \text{yaw command}, \text{collective specific force command}]^T
$$

where $a \in [a_{min}, a_{max}]$ is the scalar collective specific force in $\text{m/s}^2$ ($a \approx g$ at hover).

---

### 2.2 Continuous Nonlinear Multirotor Dynamics

$$
\begin{aligned}
\dot{\vec{p}} &= \vec{v} \\
\dot{\vec{v}} &= R(\phi, \theta, \psi) \begin{bmatrix} 0 \\ 0 \\ a \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ -g \end{bmatrix} \\
\dot{\phi} &= \frac{u_\phi - \phi}{\tau_\phi} \\
\dot{\theta} &= \frac{u_\theta - \theta}{\tau_\theta} \\
\dot{\psi} &= \dot{\psi} \\
\ddot{\psi} &= \omega_n^2 \text{wrap}(u_\psi - \psi) - 2\zeta\omega_n \dot{\psi} \\
\dot{a} &= \frac{u_a - a}{\tau_a}
\end{aligned}
$$

where $R(\phi, \theta, \psi) \in SO(3)$ is the Z-Y-X rotation matrix transforming the body thrust vector to the world ENU frame:

$$
R_{ENU/FLU} \begin{bmatrix} 0 \\ 0 \\ a \end{bmatrix} = a \begin{bmatrix} \cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi \\ \sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi \\ \cos\theta \cos\phi \end{bmatrix}
$$

---

### 2.3 Optimal Control Problem (OCP) Formulation

The Optimal Control Problem solved at every $50\text{ Hz}$ cycle ($T = 20\text{ ms}$) over horizon $N = 10$ ($T_s = 0.05\text{ s}$, $T_{horizon} = 0.5\text{ s}$) is formulated as:

$$
\min_{x_{0:N}, u_{0:N-1}} \sum_{k=0}^{N-1} \left( \|x_k - x_{ref,k}\|_{Q}^2 + \|u_k - u_{ref,k}\|_{R}^2 + W_{\Delta\psi} (\Delta u_{\psi,k})^2 \right) + \|x_N - x_{ref,N}\|_{P}^2
$$

$$\text{subject to:}$$

$$
\begin{aligned}
x_0 &= x_{\text{measured}} && \text{(Initial State Feedback)} \\
x_{k+1} &= f_{\text{ERK4}}(x_k, u_k, p) && \text{(Discrete 4th-Order Runge-Kutta Dynamics)} \\
x_{\text{lower}} &\le x_k \le x_{\text{upper}} && \text{(State Box Bounds)} \\
u_{\text{lower}} &\le u_k \le u_{\text{upper}} && \text{(Control Box Bounds)} \\
|\Delta u_k| &\le \Delta u_{\text{max}} && \text{(Slew-Rate Constraints: } \Delta u_k = u_k - u_{k-1} \text{)} \\
z_{B,z}(x_k) &\ge \cos(\theta_{\text{max}}) && \text{(Nonlinear Full-Envelope Tilt Cone Constraint)}
\end{aligned}
$$

#### Trigonometric Cost Residuals (Eliminating Yaw Singularities):
To avoid discontinuous angle jumps at $\pm \pi$, yaw tracking errors are expressed in trigonometric residual space:

$$
\|x_{yaw} - x_{ref,yaw}\|_{Q}^2 = Q_{yaw} \left[ (\sin\psi - \sin\psi_{ref})^2 + (\cos\psi - \cos\psi_{ref})^2 \right]
$$

$$
\|u_{yaw} - u_{ref,yaw}\|_{R}^2 = R_{yaw} \left[ (\sin u_\psi - \sin u_{\psi,ref})^2 + (\cos u_\psi - \cos u_{\psi,ref})^2 \right]
$$

---

## 3. Repository Structure

```text
mpc_control/
├── include/mpc_controller/
│   ├── solver/                      # TMPC Solver Port, Types & acados Adapter
│   │   ├── tpmc_solver.hpp
│   │   ├── acados_tpmc_solver.hpp
│   │   ├── tpmc_types.hpp
│   │   ├── tpmc_model.hpp
│   │   ├── tpmc_constraints.hpp
│   │   └── tpmc_reference.hpp
│   ├── controller/                  # Control Algorithms & Safety Filters
│   │   ├── command_safety_limiter.hpp
│   │   ├── force_attitude_mapping.hpp
│   │   ├── geometric_controller.hpp
│   │   ├── collective_force_filter.hpp
│   │   └── reference_model.hpp
│   ├── mission/                     # Trajectory Generation & Waypoints
│   │   ├── mission_parser.hpp
│   │   └── minimum_time_trajectory.hpp
│   └── bridge/                      # Sensor State Ingestion & Framing
│       └── state_bridge.hpp
│
├── src/
│   ├── solver/                      # Implementation of solver domain & acados bridge
│   ├── controller/                  # ROS 2 control nodes (mpc_controller_node, px4_attitude_mode_node)
│   ├── mission/                     # reference_generator_node
│   └── bridge/                      # vehicle_state_bridge_node
│
├── scripts/
│   ├── analysis/                    # Validation analysis & plotting scripts
│   ├── recording/                   # High-frequency telemetry CSV recorder
│   └── execution/                   # PID mission executor & CPU stress generator
│
├── tools/acados/                    # Symbolic CasADi OCP model & C-code generator
├── test/
│   ├── cpp/                         # C++ unit tests (tpmc_core_test.cpp)
│   └── python/                      # Python gate tests (analyze_validation_run_test.py)
│
├── config/                          # YAML parameters & benchmark mission files
├── launch/                          # ROS 2 launch scripts
├── docs/                            # Comprehensive engineering documentation suite
└── validation_runs/                 # Archived benchmark datasets and reports
```

---

## 4. Build and Quickstart

### 4.1 Prerequisites
* **OS**: Ubuntu 24.04 LTS / 22.04 LTS
* **ROS 2**: Jazzy Jalisco (or Iron / Humble)
* **PX4**: PX4-Autopilot v1.14+ SITL with Gazebo Harmonic
* **Libraries**: `Eigen3`, `acados`, `CasADi`, `px4_msgs`, `px4_ros2_cpp`

### 4.2 Build Workflow
```bash
# 1. Source ROS 2 and px4_msgs
source /opt/ros/jazzy/setup.bash
source install/px4_msgs/local_setup.bash

# 2. (Optional) Regenerate acados C solver if OCP model equations change
python3 tools/acados/generate_tpmc_solver.py --output-directory build/acados_tpmc

# 3. Build release package
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
source install/setup.bash
```

### 4.3 Running Unit & Python Tests
```bash
# Run C++ Core Math & Solver Tests
colcon test --packages-select mpc_controller && colcon test-result --verbose

# Run Python Validation Tests
pytest test/
```

---

## 5. SITL Flight Simulation & Benchmark Validation

### 5.1 Running Autonomous Mission
To launch SITL simulation with the **Obstacle Slalom** benchmark track:

```bash
# 1. Start PX4 SITL + MicroXRCE-DDS + ROS 2 Controller Nodes
SIM_LOG_XTERM=0 ROS_LAUNCH_ARGS="mission_file_path:=$PWD/config/missions/benchmark_obstacle_slalom.json" make sim

# 2. Trigger autonomous sequence (Arm -> Takeoff -> Hold -> Mission -> Land -> Disarm)
make mission-execute

# 3. Stop simulation daemon
make stop

# 4. Generate automated 11-Gate Validation Report
make validation-report MISSION_JSON=config/missions/benchmark_obstacle_slalom.json
```

---

## 6. The 11 Strict Validation Gates

The validation script [`scripts/analysis/analyze_validation_run.py`] enforces 11 criteria for flight certification:

| Gate Name | Measured Metric | Threshold | Real-World Benchmark Result | Status |
| :--- | :--- | :--- | :---: | :---: |
| `no_qp_fallback` | Count of QP solver fallbacks | **$= 0$** | **0** | ✅ **PASS** |
| `no_solver_failure` | Count of SQP solver failures / MINSTEP | **$= 0$** | **0** | ✅ **PASS** |
| `deadline_miss_rate` | Percentage of solves exceeding $18\text{ ms}$ | **$\le 1.0\%$** | **0.00%** | ✅ **PASS** |
| `worst_case_timing` | Maximum recorded end-to-end latency | **$\le 18.0\text{ ms}$** | **8.20 ms** | ✅ **PASS** |
| `hover_tilt_stability` | Roll/Pitch Euler std during hover | **$\le 5.0^\circ$** | **0.51° / 1.99°** | ✅ **PASS** |
| `maximum_tilt` | Maximum vehicle tilt angle during flight | **$\le 40.0^\circ$** | **20.61°** | ✅ **PASS** |
| `mission_xy_tracking` | Horizontal position tracking RMSE & Max | **RMSE $\le 1.0\text{ m}$, Max $\le 3.0\text{ m}$** | **0.402 m / 0.882 m** | ✅ **PASS** |
| `altitude_deviation` | Maximum vertical position tracking error | **$\le 2.0\text{ m}$** | **0.138 m** | ✅ **PASS** |
| `mission_velocity_tracking` | Translational velocity vector RMSE | **RMSE $\le 1.0\text{ m/s}$, Max $\le 3.0\text{ m/s}$** | **0.150 m/s / 0.345 m/s** | ✅ **PASS** |
| `motor_saturation` | Actuator motor PWM saturation percentage | **$= 0.000\%$** | **0.000%** | ✅ **PASS** |
| `minimum_obstacle_clearance`| Minimum distance to geometric obstacles | **$\ge \text{margin}$** | **N/A (Open track)** | ➖ **N/A** |
| **OVERALL** | **Complete Multi-Gate Audit** | — | — | ✅ **ALL PASS** |

---

## 7. Documentation Suite

Full technical specifications, architectural diagrams, tuning protocols, and telemetry registries are available in the [`docs/`](docs/) directory:

* **Architecture**:
  * [System Architecture Map (SYSTEM_MAP.md)](docs/architecture/SYSTEM_MAP.md)
  * [Runtime Signal Flow & Topic Contracts (DATA_FLOW.md)](docs/architecture/DATA_FLOW.md)
  * [Module & Executable Registry (MODULE_REGISTRY.md)](docs/architecture/MODULE_REGISTRY.md)
* **Operations**:
  * [Build & Environment Setup (BUILD_AND_RUN.md)](docs/operations/BUILD_AND_RUN.md)
  * [Command Reference & Provenance (COMMAND_REFERENCE.md)](docs/operations/COMMAND_REFERENCE.md)
  * [SITL Operational Runbook (SITL_RUNBOOK.md)](docs/operations/SITL_RUNBOOK.md)
* **Tuning**:
  * [Staged Tuning Playbook (TUNING_PLAYBOOK.md)](docs/tuning/TUNING_PLAYBOOK.md)
  * [Parameter Reference & Configuration Audit (PARAMETER_REFERENCE.md)](docs/tuning/PARAMETER_REFERENCE.md)
* **Validation**:
  * [Validation Plan & Ladder (VALIDATION_PLAN.md)](docs/validation/VALIDATION_PLAN.md)
  * [Test Matrix & Pyramid (TEST_MATRIX.md)](docs/validation/TEST_MATRIX.md)
  * [Pass/Fail Gate Specifications (PASS_FAIL_CRITERIA.md)](docs/validation/PASS_FAIL_CRITERIA.md)
* **Data**:
  * [Data Dictionary & Telemetry Columns (DATA_DICTIONARY.md)](docs/data/DATA_DICTIONARY.md)
  * [Reproducibility & Run Directory Anatomy (REPRODUCIBILITY.md)](docs/data/REPRODUCIBILITY.md)
