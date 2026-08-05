# Implementation Plan — MPC Virtual Trajectory Generator + PX4 Internal Controller

**Project name:** `mpc_trajectory_control`  
**Primary objective:** develop an MPC module that converts a preloaded/user-provided reference trajectory into a dynamically feasible virtual trajectory, then let the existing PX4 position/velocity/attitude/rate control stack track that trajectory.  
**Initial target:** PX4 SITL with Gazebo `gz_x500` model.  
**Integration order:** standalone core → mathematical simulation → ROS 2 wrapper → PX4 SITL → fault testing → KR adapter → HIL/real flight later.  
**Primary design principle:** separate trajectory shaping from vehicle stabilization and do not introduce a custom SO(3) controller in the first implementation.

---

## 1. Final architecture decision

The project uses the following control architecture:

```text
Preloaded / user trajectory
p_ref(t), v_ref(t), a_ref(t), yaw_ref(t)
                  |
                  v
        MPC virtual trajectory generator
        internal state: [p_m, v_m, a_m]
                  |
                  v
Feasible trajectory command
p_d(t), v_d(t), a_d(t), yaw_d(t)
                  |
                  v
PX4 internal position/velocity controller
                  |
                  v
PX4 attitude controller
                  |
                  v
PX4 angular-rate controller
                  |
                  v
PX4 control allocation + simulated motors
                  |
                  v
Gazebo gz_x500 plant
```

### 1.1 Meaning of the MPC block

The MPC block is **not a state estimator** and is **not the direct low-level UAV controller**.

Its responsibilities are:

- sample a known future reference over the prediction horizon;
- reshape an abrupt or dynamically infeasible reference;
- generate a smooth and feasible virtual trajectory;
- enforce configured velocity, acceleration and input/jerk constraints;
- output position, velocity and acceleration setpoints for PX4;
- expose the predicted trajectory for diagnostics and later collision checking.

Recommended terminology:

- `MPC virtual trajectory generator`;
- `MPC trajectory tracker`;
- `reference governor based on MPC`.

Avoid calling it an “estimator”.

### 1.2 Responsibility of PX4

PX4 remains responsible for:

- position and velocity feedback;
- acceleration feedforward handling;
- acceleration/thrust-vector conversion;
- attitude control;
- angular-rate control;
- actuator allocation;
- low-level mode and failsafe behavior.

A custom SO(3) controller is outside the first project scope. It may be added later only if flight-envelope or tracking evidence shows that the PX4 inner control stack is the limiting factor.

---

## 2. Explicit project scope

### 2.1 Included in the first complete project

1. Pure C++ MPC trajectory core.
2. Three translational axes with virtual state `[position, velocity, acceleration]`.
3. Preloaded references with position, velocity, acceleration and optional yaw.
4. Correct sampling of the reference on the solver prediction grid.
5. Velocity, acceleration and solver-input/jerk constraints.
6. Solver backend abstraction.
7. Characterization of `mrs_mpc_solvers` before relying on its semantics.
8. Deterministic unit, contract, property and regression tests.
9. Mathematical model-in-the-loop simulation.
10. ROS 2 wrapper and diagnostics.
11. PX4 v1.17-compatible ROS 2 adapter.
12. Gazebo SITL tests using `gz_x500`.
13. Baseline comparison against direct PX4 trajectory tracking without MPC shaping.
14. Fault tests for stale input, solver failure, timing failure, estimator reset and Offboard loss.
15. KR integration only after standalone SITL acceptance.
16. Reproducible project packaging, CI, version pinning and test artifacts.

### 2.2 Not included in the first implementation

- custom SO(3) controller;
- custom attitude/rate controller;
- thrust model or motor mixer;
- obstacle avoidance;
- multi-UAV collision avoidance;
- online path planner;
- mapping;
- direct actuator control;
- automatic takeoff/landing by MPC;
- real-flight qualification;
- HIL qualification;
- mass estimator or disturbance observer;
- integral augmentation inside the MPC baseline;
- modification of the KR planner or mission state machine.

These items belong to later milestones and must not block the initial MPC trajectory project.

---

## 3. Requirements and traceability

Create `docs/requirements.md` before implementation.

Every requirement must have:

- unique ID;
- rationale;
- measurable acceptance criterion;
- linked design component;
- linked test case;
- result artifact.

Example:

```text
REQ-TRAJ-001
The MPC output velocity shall not exceed configured per-axis limits by more
than numerical tolerance epsilon_v.

Verified by:
TEST-UNIT-LIMITER-001
TEST-MIL-CONSTRAINT-002
TEST-SITL-LINE-004
```

### 3.1 Initial functional requirements

- `REQ-FUNC-001`: load or receive a time-parameterized reference trajectory.
- `REQ-FUNC-002`: sample the reference according to the exact solver grid.
- `REQ-FUNC-003`: produce `p_d`, `v_d`, `a_d`, yaw and predicted horizon.
- `REQ-FUNC-004`: support hold, line, polynomial trajectory and analytic circle fixtures.
- `REQ-FUNC-005`: initialize the virtual trajectory state from the measured UAV state during activation.
- `REQ-FUNC-006`: maintain virtual state internally during normal operation.
- `REQ-FUNC-007`: detect excessive real-to-virtual tracking error.
- `REQ-FUNC-008`: reset or re-anchor only through an explicit state-machine policy.

### 3.2 Initial performance requirements

Initial values are provisional and must be confirmed by benchmark:

```text
Candidate MPC rates:          50 Hz and 100 Hz
Initial selected rate:        50 Hz
Heartbeat rate:               10–20 Hz
P99 solve-time budget:        < 50% of MPC period
Maximum solve time:           < 80% of MPC period
Nominal SITL hover duration:  60 s
Nominal repeat count:         at least 10 runs
Long SITL soak:               at least 1 h
```

Tracking limits must be defined per test trajectory rather than as one universal number.

### 3.3 Safety requirements

- no automatic arming in initial tests;
- no setpoint publication while inactive or invalid;
- non-finite data must never reach PX4;
- only one active Offboard setpoint owner;
- stale state/reference must transition deterministically;
- excessive tracking error must stop trajectory progression or trigger fallback;
- solver failure must not replay an old command indefinitely;
- estimator reset must reset relevant warm-start and anchoring state;
- Offboard loss behavior must be delegated to a documented PX4 failsafe configuration.

---

## 4. Repository structure

```text
mpc_trajectory_control/
├── README.md
├── LICENSE
├── implementation_plan.md
├── CMakeLists.txt
├── cmake/
│   └── warnings.cmake
├── docs/
│   ├── requirements.md
│   ├── architecture.md
│   ├── solver_contract.md
│   ├── frame_conventions.md
│   ├── safety_analysis.md
│   ├── test_strategy.md
│   ├── tuning_guide.md
│   └── adr/
├── mpc_trajectory_core/
│   ├── include/mpc_trajectory_core/
│   │   ├── types.hpp
│   │   ├── config.hpp
│   │   ├── controller.hpp
│   │   ├── solver_backend.hpp
│   │   ├── validators.hpp
│   │   ├── trajectory_sampler.hpp
│   │   ├── virtual_state.hpp
│   │   └── diagnostics.hpp
│   └── src/
├── mpc_mrs_backend/
│   ├── include/mpc_mrs_backend/
│   └── src/
├── test/
│   ├── unit/
│   ├── solver_contract/
│   ├── model_verification/
│   ├── property/
│   ├── regression/
│   ├── mil/
│   ├── ros/
│   ├── sitl/
│   ├── fixtures/
│   └── golden/
├── benchmark/
├── config/
│   ├── conservative_50hz.yaml
│   └── candidate_100hz.yaml
├── tools/
│   ├── generate_reference.py
│   ├── analyze_mpc_output.py
│   ├── analyze_sitl_run.py
│   ├── compare_baseline.py
│   └── generate_test_report.py
├── ros2/
│   ├── mpc_trajectory_ros/
│   ├── mpc_trajectory_px4/
│   └── mpc_trajectory_kr_adapter/
└── manifests/
    ├── dependencies.repos
    └── tested_versions.yaml
```

The KR adapter directory must not be implemented until the standalone SITL gate passes.

---

## 5. Core data contract

### 5.1 Reference input

```cpp
struct ReferencePoint {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  double yaw_rad;
  double yaw_rate_rad_s;
  double time_from_start_s;
};

struct ReferenceHorizon {
  std::vector<ReferencePoint> points;
};
```

The trajectory loader is outside the core. The core receives validated C++ data only.

### 5.2 Virtual trajectory state

```cpp
struct AxisState {
  double position;
  double velocity;
  double acceleration;
};

struct VirtualTrajectoryState {
  std::array<AxisState, 3> axis;
  double yaw_rad;
  double yaw_rate_rad_s;
  double trajectory_time_s;
};
```

### 5.3 Output

```cpp
struct TrajectoryCommand {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  double yaw_rad;
  double yaw_rate_rad_s;
  bool valid;
  FailureReason failure_reason;
};

struct MpcUpdateResult {
  TrajectoryCommand command;
  std::vector<TrajectoryCommand> prediction;
  MpcDiagnostics diagnostics;
};
```

The old `AccelerationCommand`-only design is rejected because the selected architecture requires position, velocity and acceleration trajectory outputs.

### 5.4 Core API

```cpp
class MpcTrajectoryCore {
public:
  ConfigureResult configure(const MpcConfig &config);

  ActivationResult activate(
      const VehicleState &measured_state,
      double reference_time_s);

  MpcUpdateResult update(
      const ReferenceHorizon &reference_horizon,
      double update_time_s);

  void reset(
      const VehicleState &measured_state,
      double reference_time_s);

  bool configured() const noexcept;
  bool active() const noexcept;
};
```

The normal `update()` path advances the virtual state and does not overwrite it with the measured UAV state every cycle.

---

## 6. Virtual-state policy

### 6.1 Activation

At handover:

```text
p_virtual <- measured position
v_virtual <- measured velocity
a_virtual <- validated acceleration estimate or zero
```

### 6.2 Normal operation

```text
x_virtual[k+1] = A x_virtual[k] + B u[k]
```

The virtual trajectory is generated continuously from the solver and model.

### 6.3 Measured-state use

Measured state is used for:

- activation;
- monitoring tracking error;
- determining whether PX4 follows the virtual trajectory;
- detecting excessive divergence;
- deciding pause, hold, reset or fallback.

It is not used to silently replace the virtual state on every solver cycle.

### 6.4 Excessive tracking error policy

Define configurable thresholds:

```text
warning tracking error
abort tracking error
consecutive-warning duration
consecutive-abort duration
```

Suggested state-machine actions:

```text
NORMAL
  -> PAUSE_REFERENCE when warning persists
  -> HOLD when tracking error remains excessive
  -> FALLBACK when hold cannot be maintained or state becomes invalid
```

Do not continue advancing a virtual trajectory far ahead of the real UAV.

---

## 7. Solver backend characterization

Treat `mrs_mpc_solvers` as a third-party component with incomplete external contract documentation.

Before controller implementation, determine through source inspection and executable tests:

1. physical meaning of `getFirstControlInput()`;
2. physical meaning of `setLastInput()`;
3. exact state order;
4. exact input order;
5. exact units;
6. horizon length and generated solver limitations;
7. interpretation of `dt1`, `dt2`, `p1`, `p2`;
8. order and meaning of `setLimits()` parameters;
9. status and failure reporting;
10. infeasible behavior;
11. NaN and dimension-error behavior;
12. exception behavior;
13. allocation behavior during solve;
14. thread safety;
15. deterministic behavior and tolerances.

Required artifacts:

```text
docs/solver_contract.md
manifests/tested_versions.yaml
test/solver_contract/*
test/golden/*
```

No later milestone may assume that the solver input is jerk or acceleration until tests prove it.

---

## 8. Reference handling

### 8.1 Supported initial reference sources

- analytic hold;
- analytic line;
- polynomial line;
- minimum-jerk fixture;
- analytic circle;
- CSV/YAML preloaded trajectory for test tooling.

### 8.2 Sampling rule

At update time `t_k`, sample the reference at the exact solver prediction times:

```text
t_k + tau_0
t_k + tau_1
...
t_k + tau_N
```

Do not assume uniform time if the solver uses a two-stage or nonuniform grid.

### 8.3 Input classification

Distinguish:

- malformed reference: NaN, wrong length, decreasing time → reject;
- dynamically inconsistent reference: position/velocity/acceleration mismatch → warn or regenerate derivatives according to policy;
- physically infeasible reference: exceeds configured flight envelope → MPC reshapes it if within configured shaping policy;
- unsafe reference: exceeds absolute project safety envelope → reject before MPC.

### 8.4 End-of-trajectory behavior

The final reference point must become an explicit hold horizon:

```text
position = final position
velocity = zero
acceleration = zero
yaw = final yaw
```

The trajectory must not extrapolate undefined data after the end.

---

## 9. Frame conventions

### 9.1 Core policy

The core is axis-generic and performs no hidden frame conversion.

State and reference supplied to the core must share:

- coordinate frame;
- axis order;
- units;
- sign convention;
- timestamp domain.

### 9.2 PX4 adapter policy

Use PX4 local NED at the PX4 boundary:

```text
X = North
Y = East
Z = Down
```

If source references are ENU, conversion must occur in one dedicated module with exhaustive tests.

Required tests:

- +X ENU to +Y NED mapping;
- +Y ENU to +X NED mapping;
- +Z ENU to -Z NED mapping;
- yaw conversion;
- yaw wraparound;
- velocity and acceleration conversion;
- no double conversion.

---

## 10. PX4 setpoint boundary

The first implementation sends the virtual trajectory to PX4 through trajectory setpoints:

```text
position     = p_d
velocity     = v_d
acceleration = a_d
yaw          = yaw_d
yawspeed     = yaw_rate_d
```

PX4 provides the feedback controller. Therefore, no external position PID is active in the baseline architecture.

### 10.1 Ownership rule

Exactly one component may publish the active Offboard setpoint stream.

When MPC output is active, disable any other KR/SO3/MAVROS path that owns the same setpoint boundary.

### 10.2 Heartbeat separation

Use independent timers:

```text
Offboard heartbeat:  10–20 Hz
MPC solve:           50 Hz initially
Trajectory setpoint: same rate as MPC output
Diagnostics:         10–20 Hz
```

Heartbeat must not depend on solver completion.

---

## 11. Development milestones

## M-1 — Requirements, architecture and safety baseline

Deliverables:

- requirements and traceability matrix;
- architecture diagram;
- frame convention document;
- initial hazard analysis;
- exact supported PX4/ROS/Gazebo target matrix;
- acceptance metrics for each test trajectory.

Gate:

- every requirement is measurable;
- control boundary is explicitly approved;
- MPC is documented as virtual trajectory generator, not direct motor controller.

## M0 — Dependency and toolchain lock

Deliverables:

- exact `mrs_mpc_solvers` commit;
- exact PX4 firmware commit/tag;
- exact `px4_msgs` commit compatible with firmware;
- ROS 2 distribution and compiler version;
- Gazebo version;
- license audit;
- dependency manifest.

Gate:

- clean reproducible build from manifest;
- no floating branches in release configuration.

## M1 — Solver contract

Deliverables:

- solver backend interface;
- minimal MRS backend;
- executable characterization tests;
- solver contract document;
- golden outputs.

Gate:

- solver input/output and time grid are proven;
- all solver failures map to project failure states.

## M2 — Core types, validation and trajectory sampling

Deliverables:

- types and configuration;
- horizon sampler;
- validators;
- virtual-state container;
- scripted solver backend;
- unit tests.

Gate:

- invalid data never reaches the solver;
- reference sampling matches solver grid exactly;
- frame-free core is deterministic.

## M3 — MPC virtual trajectory core

Deliverables:

- configure/activate/update/reset implementation;
- virtual-state propagation;
- command and predicted-horizon output;
- failure handling;
- diagnostics.

Gate:

- core passes all L1–L3 tests;
- sanitizer clean;
- no exception crosses update boundary;
- no uncontrolled dynamic allocation after activation where practical.

## M4 — Mathematical simulation and tuning

Deliverables:

- verified discrete model;
- reference generator fixtures;
- model-in-the-loop runner;
- 50 Hz and 100 Hz benchmarks;
- conservative initial tuning profile.

Gate:

- trajectory constraints respected;
- hold and final hold converge;
- no divergence under bounded uncertainty tests;
- timing budget satisfied.

## M5 — ROS 2 wrapper

Deliverables:

- lifecycle node;
- normalized input/output messages;
- parameter validation;
- timeout handling;
- diagnostics and optional predicted-path publication;
- launch tests.

Gate:

- no valid output while inactive/error;
- deterministic reset and timeout behavior;
- no PX4 dependency in core tests.

## M6 — PX4 message integration, no arm

Deliverables:

- PX4 adapter;
- NED mapping;
- Offboard heartbeat;
- trajectory setpoint publication;
- estimator-validity handling;
- reset-counter handling;
- no-arm launch tests.

Gate:

- correct message fields and signs confirmed from ROS topics/bags;
- no automatic arm;
- no competing setpoint publisher.

## M7 — Gazebo `gz_x500` SITL nominal tests

Deliverables:

- reproducible SITL launch;
- direct PX4 baseline path;
- MPC-shaped path;
- hover, axis, line, diagonal, circle and final-hold tests;
- ULog, ROS bag and machine-readable report.

Gate:

- nominal suite passes acceptance thresholds;
- no Offboard dropouts;
- no NaN/Inf;
- repeated runs are stable.

## M8 — Gazebo `gz_x500` fault and robustness tests

Deliverables:

- stale reference;
- stale vehicle state;
- delayed state;
- out-of-order timestamps;
- solver failure injection;
- deadline miss;
- node restart;
- heartbeat kill;
- estimator reset;
- excessive real-to-virtual tracking error;
- CPU load and timing jitter test.

Gate:

- all transitions match the documented state machine;
- PX4 executes configured failsafe on Offboard loss;
- no uncontrolled command replay.

## M9 — KR shadow integration

Deliverables:

- KR state/reference adapter;
- shadow-mode output logging only;
- standalone-versus-KR replay comparison;
- frame and timestamp verification.

Gate:

- shadow command matches standalone output within documented tolerance.

## M10 — KR active integration

Deliverables:

- explicit controller selector;
- rollback path;
- dual-owner prevention;
- full M7/M8 suite through KR reference path.

Gate:

- no regression versus standalone path;
- planner/state-machine changes remain minimal and documented.

## M11 — Later lifecycle milestones

Not part of the initial software completion, but required before claiming full UAV product maturity:

- processor-in-the-loop on target companion computer;
- HIL with real flight controller;
- props-off bench testing;
- tethered/low-envelope flight;
- controlled flight-envelope expansion;
- release qualification;
- field monitoring and maintenance.

---

## 12. Test hierarchy

Tests are executed in order. A higher layer may not be used to hide failures in a lower layer.

## L0 — Build and static quality

Run:

```text
Debug build
Release build
-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
clang-format
clang-tidy
ASan
UBSan
```

Gate:

- zero warnings;
- zero sanitizer findings;
- reproducible build.

## L1 — Unit tests

### Configuration

- invalid horizon;
- nonpositive time steps;
- invalid weights;
- non-finite gains;
- invalid constraints;
- inconsistent solver-grid configuration.

### Reference validation

- wrong length;
- decreasing/equal timestamps;
- NaN/Inf at beginning, middle and end;
- end-of-trajectory hold;
- out-of-envelope reference.

### Virtual-state logic

- activation from measured state;
- normal propagation;
- reset;
- no silent overwrite from measured state;
- yaw wrapping.

### Frame conversion

- all ENU/NED axis directions;
- yaw conversion and wrap.

### Output validation

- finite command;
- finite prediction;
- limits respected;
- invalid solver output rejected.

Gate:

- core line coverage >= 90%;
- validator/failure branch coverage = 100%;
- no flaky result over 100 repeated runs.

## L2 — Solver contract and golden regression

Tests:

- zero state/zero reference;
- positive/negative symmetry;
- step reference;
- constant velocity;
- constant acceleration;
- saturation;
- warm start;
- reset;
- infeasible case;
- exact horizon boundary;
- X/Y identical configuration equivalence;
- deterministic output tolerance.

Gate:

- exact solver commit recorded;
- golden changes require explicit review.

## L3 — Property/randomized tests

Generate at least 10,000 bounded cases.

Invariants:

- valid output is finite;
- constraints are respected;
- no throw/crash;
- symmetric problems produce symmetric outputs within tolerance;
- zero error does not create large control action;
- prediction timestamps are strictly monotonic.

## L4 — Model verification

Verify:

- discrete matrices against analytical or numerical integration;
- `A -> I` and `B -> 0` as `dt -> 0`;
- multi-step prediction;
- exact solver time grid;
- model response under parameter variation;
- selected input semantics.

## L5 — Model-in-the-loop trajectory tests

Test references:

1. hold;
2. X step;
3. Y step;
4. Z step;
5. ramp;
6. polynomial/minimum-jerk line;
7. diagonal 3D;
8. circle;
9. abrupt infeasible command;
10. final hold;
11. changing constraints;
12. solver failure.

Metrics:

- shaping error `p_ref - p_d`;
- virtual velocity/acceleration/jerk;
- constraint violations;
- final-state error;
- command total variation;
- solve-time mean/P95/P99/max.

## L6 — Robustness and Monte Carlo

Vary:

- initial virtual state;
- model parameters;
- time step;
- reference delay;
- solver timing jitter;
- noise injected into activation state;
- constraint margins.

Report:

- success rate;
- worst case;
- P95/P99;
- number of failure transitions.

## L7 — ROS wrapper tests

- lifecycle transitions;
- parameter updates only while inactive;
- stale input;
- out-of-order input;
- simulated-time pause/jump;
- node restart;
- wrong QoS;
- diagnostics rate;
- predicted-path publication.

## L8 — PX4 no-arm interface tests

- heartbeat rate;
- setpoint rate;
- NED sign;
- finite fields;
- correct yaw/yaw-rate;
- estimator validity mapping;
- reset-counter mapping;
- no competing publisher;
- no automatic arming.

## L9 — PX4 Gazebo SITL nominal tests

See Section 13.

## L10 — PX4 Gazebo fault tests

See Section 14.

## L11 — Soak and performance tests

- SITL run for at least one hour;
- CPU and memory trend;
- missed deadlines;
- DDS reconnect behavior;
- solver outliers;
- no memory leak.

---

## 13. `gz_x500` PX4 SITL nominal test plan

### 13.1 Test environment

Record for every run:

- PX4 commit/tag;
- `px4_msgs` commit;
- ROS 2 distribution;
- Gazebo version;
- `gz_x500` model version;
- MPC commit;
- solver commit;
- configuration hash;
- PX4 parameter file hash;
- random seed;
- machine hardware and CPU load.

### 13.2 Test preparation

1. Launch PX4 SITL with `gz_x500`.
2. Confirm vehicle state topics are valid.
3. Confirm clocks and timestamps progress correctly.
4. Start MPC stack inactive.
5. Confirm no Offboard setpoint owner conflict.
6. Use PX4 normal controller to take off and hold.
7. Start heartbeat prestream.
8. Activate MPC trajectory publication only after all entry checks pass.

### 13.3 Nominal sequence

#### SITL-NOM-001 — Message-only test

- vehicle remains disarmed;
- publish hold trajectory;
- inspect `TrajectorySetpoint` and `OffboardControlMode`;
- verify NED signs and no NaN in active fields.

#### SITL-NOM-002 — Handover in hover

- take off using normal PX4 workflow;
- hold stable;
- initialize virtual state from measured state;
- switch to MPC trajectory source;
- verify bounded command discontinuity.

#### SITL-NOM-003 — Hover 60 s

- constant hold reference;
- zero Offboard dropout;
- bounded tracking error;
- no repeated virtual-state reset.

#### SITL-NOM-004/005/006 — X/Y/Z smooth motion

- one axis at a time;
- low speed and acceleration;
- verify sign and final hold.

#### SITL-NOM-007 — Straight line

- use known polynomial reference;
- compare raw reference, MPC trajectory and actual UAV path.

#### SITL-NOM-008 — Diagonal 3D

- verify simultaneous axes;
- monitor PX4 tilt and acceleration saturation.

#### SITL-NOM-009 — Slow circle

- constant altitude;
- optional yaw tangent or fixed yaw;
- inspect phase lag and radial error.

#### SITL-NOM-010 — Infeasible step reference

- raw trajectory contains a large position step;
- MPC must produce a smooth constrained trajectory;
- PX4 must never receive the raw step directly.

#### SITL-NOM-011 — End-of-trajectory hold

- verify reference stops advancing;
- velocity and acceleration converge to zero;
- UAV holds final point.

### 13.4 Required metrics

Separate three errors:

```text
Shaping error: e_shape = p_ref - p_d
Tracking error: e_track = p_d - p_uav
Total mission error: e_total = p_ref - p_uav
```

Also record:

- velocity and acceleration limits;
- estimated jerk/input;
- PX4 saturation indicators where available;
- attitude/tilt;
- solve timing;
- setpoint age;
- state age;
- Offboard state;
- estimator reset counters;
- mode transitions.

### 13.5 Baseline comparison

Run the same reference and initial condition with:

```text
Baseline A: reference sent directly to PX4
Candidate B: reference shaped by MPC, then sent to PX4
```

Compare:

- tracking RMSE/P95/max;
- overshoot;
- setpoint smoothness;
- velocity/acceleration constraint violations;
- settling time;
- PX4 saturation duration;
- CPU cost;
- solver deadline margin.

MPC is not considered beneficial merely because it runs. The report must show where it improves feasibility, smoothness or constraint handling and what delay it introduces.

---

## 14. `gz_x500` PX4 SITL fault test plan

### SITL-FAULT-001 — Reference stale

Expected:

- no undefined extrapolation;
- transition to explicit hold or configured fallback;
- diagnostics state recorded.

### SITL-FAULT-002 — Vehicle state stale

Expected:

- no solver update using stale state-dependent activation/monitoring data;
- no new valid command;
- fallback request.

### SITL-FAULT-003 — Solver failure

Expected:

- invalid result;
- optional last valid command for at most one cycle if policy allows;
- repeated failures trigger hold/fallback.

### SITL-FAULT-004 — Solver deadline miss

Expected:

- heartbeat continues;
- timing fault counter increments;
- repeated misses cause deterministic degraded/fallback state.

### SITL-FAULT-005 — Heartbeat node/process killed

Expected:

- PX4 leaves Offboard according to configured failsafe;
- behavior confirmed in ULog.

### SITL-FAULT-006 — MPC process restart

Expected:

- no uncontrolled setpoint jump;
- reactivation requires prestream and explicit state initialization.

### SITL-FAULT-007 — Estimator reset

Expected:

- reset counters detected;
- virtual state and warm start handled according to policy;
- no large command spike.

### SITL-FAULT-008 — Tracking error exceeds warning threshold

Expected:

- pause reference progression;
- warning diagnostic;
- trajectory does not continue moving away.

### SITL-FAULT-009 — Tracking error exceeds abort threshold

Expected:

- explicit hold/fallback;
- no silent re-anchor while still active.

### SITL-FAULT-010 — CPU load/jitter

Expected:

- P99 and maximum timing remain within gate or controlled fallback occurs.

### SITL-FAULT-011 — Out-of-order timestamps

Expected:

- sample rejected;
- no reference-time rollback.

### SITL-FAULT-012 — Non-finite input/output

Expected:

- immediate drop;
- fatal diagnostic;
- no non-finite PX4 message field.

---

## 15. State machine

Recommended runtime state machine:

```text
UNCONFIGURED
    |
    v
INACTIVE
    |
    | activate + valid state/reference
    v
PRESTREAM
    |
    | heartbeat and entry conditions satisfied
    v
ACTIVE
    |
    +--> PAUSED        tracking warning/reference pause
    |
    +--> HOLD          stale reference/recoverable fault
    |
    +--> FALLBACK      repeated solver/timing/state failure
    |
    +--> ERROR         non-finite/configuration/internal fatal fault
```

Transitions must be unit-tested and integration-tested.

Core and runtime responsibilities remain separate:

- core returns valid/invalid trajectory and diagnostics;
- wrapper/runtime decides pause, hold, fallback or PX4 mode transition.

---

## 16. Safety analysis

Create at minimum a lightweight hazard table and FMEA.

Examples:

| Hazard | Cause | Effect | Detection | Mitigation |
|---|---|---|---|---|
| Z sign reversed | ENU/NED error | climb instead of descend | axis test/no-arm inspection | centralized conversion + test |
| Virtual path runs ahead | poor real tracking | large catch-up command | tracking monitor | pause/hold/fallback |
| Dual setpoint owner | KR/PX4 bridge conflict | undefined command stream | ROS graph assertion | exclusive ownership |
| Solver overrun | CPU load | setpoint jitter | timing monitor | independent heartbeat + fallback |
| Estimator reset | local-frame jump | command discontinuity | reset counters | reset/re-anchor policy |
| Stale reference | source stops | replay old trajectory | timestamp monitor | explicit hold |
| Non-finite solver output | numerical fault | invalid command | output validation | drop and error |

Before later real flight, add:

- manual override;
- geofence;
- altitude limit;
- RC kill;
- test-area limits;
- abort criteria;
- flight test cards.

---

## 17. Tuning policy

1. Do not tune before sign, frame and solver-contract tests pass.
2. Tune the virtual model in 1D first.
3. Tune X/Y symmetrically.
4. Tune Z separately.
5. Validate diagonal/3D behavior.
6. Test 50 Hz before 100 Hz.
7. Record every gain/profile with:
   - solver commit;
   - model parameters;
   - rate;
   - horizon;
   - constraints;
   - test report hash.
8. Do not add integral/disturbance compensation to hide architecture, frame or model errors.
9. Keep reserve between MPC trajectory constraints and PX4/UAV hard limits.

---

## 18. CI strategy

### Every commit

- formatting;
- lint/static analysis;
- Debug/Release build;
- unit tests;
- solver-contract tests;
- sanitizer tests;
- short MIL regression;
- coverage gate.

### Nightly

- 10,000+ randomized tests;
- 100 repeated regression runs;
- Monte Carlo MIL;
- performance benchmark and trend detection;
- one-hour soak where infrastructure allows.

### PX4 adapter PRs

- headless `gz_x500` smoke test;
- no-arm message test;
- nominal hover/axis smoke;
- fault transition subset.

### Nightly SITL

- full nominal sequence;
- fault matrix;
- baseline comparison;
- store ROS bag summary, ULog metadata and machine-readable report.

---

## 19. Definition of Done — standalone MPC trajectory core

- [ ] Requirements and traceability exist.
- [ ] Solver commit and license are pinned.
- [ ] Solver semantics are proven by executable tests.
- [ ] Core has no ROS/PX4/KR dependency.
- [ ] Reference sampling matches solver grid.
- [ ] Virtual state is distinct from measured UAV state.
- [ ] Activation/reset policy is tested.
- [ ] Output contains position, velocity, acceleration and prediction.
- [ ] Invalid/non-finite input is rejected.
- [ ] No exception crosses the update boundary.
- [ ] Unit coverage and branch gates pass.
- [ ] 10,000 randomized cases pass.
- [ ] ASan/UBSan clean.
- [ ] Model verification passes.
- [ ] MIL hold/line/diagonal/circle/final-hold pass.
- [ ] Constraint tests pass.
- [ ] 50/100 Hz timing report exists.
- [ ] Conservative profile is versioned.
- [ ] Machine-readable report is generated.

---

## 20. Definition of Done — PX4 `gz_x500` SITL

- [ ] Firmware, messages and model versions are pinned.
- [ ] NED mapping tests pass.
- [ ] No-arm message tests pass.
- [ ] Exactly one setpoint owner is active.
- [ ] Heartbeat is independent of solver timer.
- [ ] Handover discontinuity is within the specified threshold.
- [ ] Hover 60 s passes.
- [ ] X/Y/Z tests pass.
- [ ] Line/diagonal/circle/final hold pass.
- [ ] Infeasible step is reshaped correctly.
- [ ] Zero nominal Offboard dropout.
- [ ] Zero NaN/Inf in bag and ULog.
- [ ] Fault transitions pass.
- [ ] Baseline comparison report exists.
- [ ] Nominal/fault suite passes at least 10 repeated runs.
- [ ] One-hour soak shows no leak or uncontrolled timing trend.

Only after this checklist may KR become the active reference path.

---

## 21. Agent execution protocol

The implementation agent must not jump directly into writing large amounts of code. For every phase it must:

1. inspect the current repository and dependency state;
2. restate the phase objective;
3. identify missing information and assumptions;
4. compare the current state with the plan and requirements;
5. propose the smallest implementation slice;
6. show files to create or modify;
7. create tests before or together with implementation;
8. execute build/tests where tools are available;
9. report evidence, not only claims;
10. stop at the phase gate and ask for approval before widening scope.

The agent must actively correct incorrect assumptions from the user or existing code. It must explain why the assumption is incorrect and provide the safer/correct alternative.

---

# Master Prompt for the Implementation Agent

Copy the following prompt into the coding/engineering agent at the start of the project.

```text
You are the lead controls and robotics software engineer for the project
`mpc_trajectory_control`.

PROJECT GOAL
Develop an MPC virtual trajectory generator that converts a known/preloaded
reference trajectory into a smooth dynamically feasible trajectory
(position, velocity, acceleration and optional yaw). PX4's existing internal
position/velocity/attitude/rate controllers will track this trajectory.
The first target is PX4 SITL with Gazebo `gz_x500`.

ARCHITECTURE DECISION
- MPC is a trajectory generator/reference governor, not a state estimator.
- MPC is not the low-level UAV controller.
- The MPC maintains a virtual state [p, v, a].
- The measured UAV state initializes the virtual state and monitors tracking
  error, but must not silently overwrite the virtual state every cycle.
- Output to PX4 is p_d, v_d, a_d, yaw_d and yaw_rate_d.
- PX4 performs position/velocity feedback and all lower control loops.
- Do not implement a custom SO(3) controller in the initial scope.
- Do not implement an external PID in parallel with the PX4 position
  controller.
- Exactly one Offboard setpoint owner is allowed.

SOURCE OF TRUTH
1. `implementation_plan.md`
2. `docs/requirements.md`
3. approved ADRs
4. pinned dependency manifest
5. executable tests

OPERATING RULES
1. Work milestone by milestone. Do not skip gates.
2. Before coding, inspect the repository and report what exists.
3. Never assume solver semantics from function names. Characterize and test
   `mrs_mpc_solvers` first.
4. Never assume `getFirstControlInput()` means acceleration or jerk without
   executable evidence.
5. Keep the core independent of ROS, PX4 and KR.
6. Put all frame conversion in a dedicated adapter module.
7. Use local NED only at the PX4 boundary.
8. Never publish non-finite setpoints.
9. Never auto-arm in initial integration tests.
10. Keep Offboard heartbeat independent of solver execution.
11. Every code change must include or update tests.
12. Do not tune gains to hide sign, model, frame or solver-contract errors.
13. When evidence contradicts the user's assumption, clearly challenge it,
    explain the issue and propose the correct approach.
14. Do not claim a test passed unless you executed it or provide a clear
    command and mark it as not yet executed.
15. Preserve reproducibility: record commits, config hashes, seeds and test
    environment.

REQUIRED RESPONSE FORMAT FOR EACH PHASE
A. Phase objective
B. Current repository assessment
C. Assumptions and unresolved questions
D. Design decision for this phase
E. Files to create/modify
F. Tests to write first
G. Implementation steps
H. Commands to build/test/run
I. Expected outputs and acceptance criteria
J. Risks and rollback
K. Evidence/results
L. Gate decision: PASS / FAIL / BLOCKED
M. Recommended next phase

IMPLEMENTATION ORDER
M-1 Requirements and architecture
M0 Dependency/toolchain pinning
M1 Solver characterization
M2 Core data types, validation and reference sampler
M3 Virtual trajectory MPC core
M4 MIL model verification, trajectory tests and benchmark
M5 ROS 2 lifecycle wrapper
M6 PX4 adapter and no-arm message test
M7 Gazebo `gz_x500` nominal SITL
M8 Gazebo `gz_x500` fault/robustness SITL
M9 KR shadow adapter
M10 KR active integration

TESTING ORDER
L0 Build/static/sanitizers
L1 Unit tests
L2 Solver contract/golden regression
L3 Property/randomized tests
L4 Model verificationSession
L5 MIL trajectory tests
L6 Monte Carlo robustness
L7 ROS wrapper tests
L8 PX4 no-arm interface tests
L9 SITL nominal tests
L10 SITL fault tests
L11 soak/performance tests

IMPORTANT METRICS
Always separate:
- shaping error: p_ref - p_d
- tracking error: p_d - p_uav
- total mission error: p_ref - p_uav
Also report constraint violations, setpoint smoothness, solve time
mean/P95/P99/max, setpoint age, state age, Offboard dropouts, estimator
resets and fallback transitions.

INITIAL TASK
Start with M-1 only.
1. Inspect the repository.
2. Create or refine `docs/requirements.md`, `docs/architecture.md`,
   `docs/frame_conventions.md`, `docs/safety_analysis.md`, and ADR-000 for
   the architecture decision.
3. Produce a requirement-to-test traceability table.
4. Do not implement the MPC controller yet.
5. Present the gate result and wait for approval before M0.
```

---

## 22. Prompt template for each automatic sub-plan

After the master prompt, use this template for an individual milestone:

```text
Execute milestone <MILESTONE_ID> from `implementation_plan.md`.

Before editing:
- inspect all relevant files;
- compare repository state against the milestone entry criteria;
- identify blockers and incorrect assumptions;
- produce a small implementation sub-plan.

During execution:
- write tests first or alongside code;
- keep changes limited to this milestone;
- run available build/test commands;
- save machine-readable outputs where required.

After execution, report:
1. files changed;
2. design decisions;
3. tests added;
4. commands executed;
5. exact results;
6. remaining risks;
7. traceability updates;
8. PASS/FAIL/BLOCKED gate decision;
9. the smallest next milestone task.

Do not begin the next milestone automatically if the current gate fails.
```

---

## 23. First recommended execution sequence

The first sessions with the agent should be:

```text
Session 1: M-1 requirements, architecture, frames and safety baseline
Session 2: M0 dependency/toolchain/version pinning
Session 3: M1 solver source inspection and characterization test plan
Session 4: M1 executable solver contract tests
Session 5: M2 core types, validation and trajectory sampler
Session 6: M3 virtual-state controller skeleton using scripted backend
Session 7: M3 actual MRS backend integration
Session 8: M4 mathematical plant/model verification
Session 9: M4 trajectory fixtures, tuning and timing benchmark
Session 10: M5 ROS wrapper
Session 11: M6 PX4 no-arm interface
Session 12+: M7/M8 incremental `gz_x500` SITL tests
```

Each session must end with a gate result and a version-control checkpoint.

---

## 24. Final project statement

The first project release is complete when it can reproducibly demonstrate:

```text
known reference trajectory
        -> MPC virtual feasible trajectory
        -> PX4 internal controller
        -> stable gz_x500 SITL flight
```

with:

- clear separation between shaping and tracking errors;
- verified solver semantics;
- documented frame conventions;
- deterministic fault behavior;
- versioned configurations;
- baseline comparison;
- complete test evidence.

SO(3), direct acceleration feedback MPC, HIL and real flight remain later extensions and must not be mixed into the first correctness milestone.