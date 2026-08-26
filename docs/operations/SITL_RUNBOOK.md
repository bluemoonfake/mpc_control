# SITL Operational Runbook (SITL_RUNBOOK.md)

This runbook specifies the step-by-step procedures for running each supported flight simulation workflow.

---

## Workflow 1: Standard TP-MPC Mission Execution & Validation

### Prerequisites:
- PX4 SITL built with Gazebo Harmonic/Garden.
- Local ROS 2 workspace built and sourced.
- Magnetometer calibration offsets clean (0.0 in `parameters.bson`).

### Step-by-Step Execution:
1. **Launch SITL with Selected Mission**:
   ```bash
   SIM_LOG_XTERM=0 ROS_LAUNCH_ARGS="mission_file_path:=$PWD/config/missions/benchmark_obstacle_slalom.json" make sim
   ```
2. **Execute Autonomous Flight**:
   ```bash
   make mission-execute
   ```
   *Behavior*:
   - PX4 completes preflight checks (`Ready for takeoff`).
   - `MpcModeExecutor` arms the vehicle and ascends to takeoff altitude ($2.5\text{ m}$).
   - MPC enters Handover Hold Gate (stabilizes for 5 cycles / $100\text{ ms}$).
   - `reference_generator_node` streams the waypoint trajectory.
   - Upon reaching the final landing waypoint, native PX4 landing mode takes over.
   - The vehicle touches down and disarms automatically.
3. **Stop Simulation Daemon**:
   ```bash
   make stop
   ```
4. **Generate Strict Validation Report**:
   ```bash
   make validation-report MISSION_JSON=config/missions/benchmark_obstacle_slalom.json
   ```

---

## Workflow 2: CPU Stress / Hardware Load Testing

### Purpose:
Validates real-time solver deadline compliance ($18.0\text{ ms}$) under intense processor contention.

### Step-by-Step Execution:
1. **Start SITL in Terminal 1**:
   ```bash
   SIM_LOG_XTERM=0 ROS_LAUNCH_ARGS="mission_file_path:=$PWD/config/missions/benchmark_obstacle_slalom.json" make sim
   ```
2. **Launch Background CPU Load in Terminal 2**:
   ```bash
   python3 scripts/execution/cpu_load.py --duration 120 --cores 2
   ```
3. **Trigger Mission Execution in Terminal 1**:
   ```bash
   make mission-execute
   ```
4. **Stop & Audit Timing Metrics**:
   ```bash
   make stop
   make validation-report MISSION_JSON=config/missions/benchmark_obstacle_slalom.json
   ```
   *Acceptance Criteria*: `worst_case_timing` must remain $\le 18.0\text{ ms}$, `deadline_miss_rate` $= 0\%$.

---

## Workflow 3: Collective Specific Force System Identification

### Purpose:
Estimates the multirotor's vertical specific force time constant $\tau_a$ and hover collective thrust $\hat{T}_{hover}$.

### Step-by-Step Execution:
1. **Start Base SITL**:
   ```bash
   SIM_LOG_XTERM=0 make sim
   ```
2. **Arm & Takeoff into Position Hold**:
   ```bash
   make arm
   make offboard
   ```
3. **Launch Identification Node**:
   ```bash
   ros2 launch mpc_controller collective_identification.launch.py
   ```
4. **Analyze Recorded Data**:
   ```bash
   make collective-identification-analysis
   ```
   *Outputs*: Populates estimated $\tau_a$ and $T_{hover}$ into `/tmp/mpc_controller_sim/collective_identification.json`.

---

## Workflow 4: PX4 Native PID Baseline Comparison

### Purpose:
Provides a direct benchmark of TP-MPC trajectory tracking versus the native PX4 Cascaded PID/PosControl stack.

### Step-by-Step Execution:
1. **Execute PID Mission**:
   ```bash
   make sim
   python3 scripts/execution/run_px4_pid_mission.py --mission config/missions/benchmark_urban_canyon.json
   make stop
   ```
2. **Execute MPC Mission**:
   ```bash
   SIM_LOG_XTERM=0 ROS_LAUNCH_ARGS="mission_file_path:=$PWD/config/missions/benchmark_urban_canyon.json" make sim
   make mission-execute
   make stop
   ```
3. **Generate Direct Comparative Benchmark Plot**:
   ```bash
   python3 scripts/analysis/compare_mission_logs.py /tmp/mpc_controller_sim/pid_run.ulg /tmp/mpc_controller_sim/mpc_run.ulg --mission config/missions/benchmark_urban_canyon.json --out mpc_vs_pid_comparison.png
   ```
