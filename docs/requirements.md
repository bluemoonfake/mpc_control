# MPC trajectory project requirements

Status: M-1 baseline. Requirements are frozen for the first standalone
implementation; numeric limits marked `provisional` require benchmark evidence
before SITL activation.

## Scope and terminology

The system is an MPC virtual trajectory generator. It receives a validated,
time-parameterized reference and produces a feasible trajectory for PX4's
internal position/velocity controller. It is not an estimator, attitude
controller, rate controller or actuator controller.

The core state is the virtual state `[position, velocity, acceleration]`.
Measured vehicle state is used for activation and monitoring; it must not
silently replace the virtual state on every update.

## Requirement conventions

Each requirement has an identifier, an acceptance criterion, a verification
method and an expected result artifact. `TBD` means an explicit unresolved
dependency, not an implicit assumption.

## Functional requirements

| ID | Requirement | Acceptance criterion | Verification | Artifact |
|---|---|---|---|---|
| REQ-FUNC-001 | Accept a validated time-parameterized reference. | Reference points contain finite position, velocity, acceleration, time and optional yaw fields. | TEST-REF-VALIDATION | Unit report |
| REQ-FUNC-002 | Sample the reference on the solver prediction grid. | Every sampled point corresponds to a documented solver time; no one-row-per-step assumption is used. | TEST-SAMPLER-GRID | Sampler report |
| REQ-FUNC-003 | Produce position, velocity, acceleration, yaw and validity state. | A valid result contains finite bounded command fields and a documented failure reason otherwise. | TEST-CORE-OUTPUT | Core test report |
| REQ-FUNC-004 | Support analytic hold, line, minimum-jerk and circle fixtures. | Each fixture has deterministic p/v/a derivatives and passes consistency checks. | TEST-FIXTURE-CONSISTENCY | Fixture report |
| REQ-FUNC-005 | Initialize the virtual state during activation. | `p_m`, `v_m` equal the measured state; acceleration follows the activation policy. | TEST-VIRTUAL-ACTIVATE | Unit report |
| REQ-FUNC-006 | Maintain virtual state during normal operation. | The update path propagates virtual state and does not overwrite it from measured state. | TEST-VIRTUAL-PROPAGATION | Unit/property report |
| REQ-FUNC-007 | Monitor real-to-virtual tracking error. | Warning and abort thresholds are configurable and reported with timestamps. | TEST-TRACKING-MONITOR | Diagnostics report |
| REQ-FUNC-008 | Apply explicit reset/re-anchor policy. | Reset occurs only through an explicit state transition and is observable in diagnostics. | TEST-RESET-POLICY | State-machine report |

## Control-boundary requirements

| ID | Requirement | Acceptance criterion | Verification | Artifact |
|---|---|---|---|---|
| REQ-BOUNDARY-001 | PX4 owns feedback position/velocity and lower loops. | No external position PID, SO(3), rate or actuator controller exists in the baseline. | TEST-ARCH-REVIEW | Architecture review |
| REQ-BOUNDARY-002 | MPC output is a trajectory command, not raw solver input. | Adapter maps the predicted `[p,v,a]` state to `p_d,v_d,a_d`; raw `u` is not mislabeled as `a_d`. | TEST-SOLVER-SEMANTICS, TEST-PX4-SETPOINT | Contract/golden report |
| REQ-BOUNDARY-003 | Exactly one active Offboard setpoint owner exists. | Ownership is documented and competing publishers are detected or disabled before activation. | TEST-SETPOINT-OWNERSHIP | ROS/PX4 graph evidence |
| REQ-BOUNDARY-004 | Core remains middleware-independent. | Core target builds without ROS 2, PX4, Gazebo or KR packages. | TEST-CORE-NO-ROS | Build log |

## Reference, frame and time requirements

| ID | Requirement | Acceptance criterion | Verification | Artifact |
|---|---|---|---|---|
| REQ-REF-001 | Reject malformed references. | NaN/Inf, wrong length, non-monotonic timestamps and invalid dimensions are rejected before solver invocation. | TEST-REF-MALFORMED | Unit report |
| REQ-REF-002 | Define end-of-trajectory behavior. | After the final point, the sampler supplies an explicit final hold with zero velocity and acceleration. | TEST-FINAL-HOLD | Sampler report |
| REQ-FRAME-001 | Keep core frame-neutral. | Core requires one declared frame and performs no hidden conversion. | TEST-FRAME-CORE | API review |
| REQ-FRAME-002 | Convert ENU to PX4 local NED exactly once. | Position, velocity, acceleration and yaw mappings pass signed-axis tests. | TEST-ENU-NED | Adapter report |
| REQ-TIME-001 | Run the initial candidate MPC loop at 50 Hz. | Measured P99 solve time is below 50% of the 20 ms period; maximum is below 80%. | TEST-TIMING-50HZ | Timing report |
| REQ-TIME-002 | Keep heartbeat independent from solver completion. | Heartbeat continues according to its own timer during a solver deadline miss. | TEST-HEARTBEAT-INDEPENDENCE | Fault report |
| REQ-TIME-003 | Reject stale or out-of-order data deterministically. | No stale reference/state is replayed indefinitely; transition and reason are logged. | TEST-STALE-DATA | Fault report |

## Safety requirements

| ID | Requirement | Acceptance criterion | Verification | Artifact |
|---|---|---|---|---|
| REQ-SAFE-001 | No automatic arming in initial tests. | No code path in the core or no-arm adapter sends an arm command. | TEST-NO-ARM | Topic/bag evidence |
| REQ-SAFE-002 | Never publish non-finite commands. | No NaN/Inf reaches the PX4 boundary; invalid output is rejected. | TEST-NONFINITE-BOUNDARY | Unit and adapter report |
| REQ-SAFE-003 | Do not publish valid commands while inactive/error. | Runtime state outside ACTIVE produces no valid trajectory command. | TEST-LIFECYCLE-OUTPUT | ROS report |
| REQ-SAFE-004 | Handle solver failure explicitly. | Failure maps to a finite, documented runtime state and does not replay an old command indefinitely. | TEST-SOLVER-FAILURE | Fault report |
| REQ-SAFE-005 | Handle estimator reset explicitly. | Reset detection causes documented re-anchor/warm-start behavior and command continuity is checked. | TEST-ESTIMATOR-RESET | SITL report |
| REQ-SAFE-006 | Stop progression on excessive tracking error. | Warning pauses or limits progression; abort enters documented hold/fallback behavior. | TEST-TRACKING-ABORT | MIL/SITL report |
| REQ-SAFE-007 | Delegate Offboard loss to documented PX4 failsafe policy. | PX4 failsafe parameters and observed transition are recorded; the MPC does not disable safety checks. | TEST-OFFBOARD-FAILSAFE | ULog/report |

## Initial operating envelope and metrics

These are starting acceptance values, not flight authorization:

- MPC candidate rates: 50 Hz selected initially, 100 Hz for later comparison.
- Heartbeat: 10–20 Hz, independent of solve timer.
- Nominal hover test: 60 s.
- Nominal repeat count: at least 10 runs at the SITL gate.
- Solver timing: P99 `< 50%` of period, maximum `< 80%` of period.
- Metrics must separate shaping error `p_ref - p_d`, tracking error
  `p_d - p_uav` and total error `p_ref - p_uav`.
- Every trajectory report includes RMSE, P95, maximum error, overshoot,
  settling time, command continuity, constraint violations and solver timing.
- Per-axis velocity, acceleration, jerk and tracking limits remain provisional
  until the M2/M4 configuration and model benchmarks exist.

## Explicit non-goals

The first implementation does not include custom SO(3), external PID,
actuator/motor commands, obstacle avoidance, online planning, automatic
takeoff/landing, HIL, real flight or KR setpoint ownership.

