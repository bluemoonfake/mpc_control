# MPC virtual trajectory architecture

Status: M-1 baseline. This document is the control-boundary authority for the
first implementation.

## System boundary

```text
validated reference trajectory
 p_ref(t), v_ref(t), a_ref(t), yaw_ref(t)
                 |
                 v
        MpcTrajectoryCore
        virtual state [p_m, v_m, a_m]
                 |
                 v
 trajectory command p_d, v_d, a_d, yaw_d, yaw_rate_d
                 |
                 v
       PX4 TrajectorySetpoint adapter
                 |
                 v
 PX4 position/velocity -> attitude -> rate -> allocation
                 |
                 v
             Gazebo gz_x500
```

The core is pure C++ and owns trajectory shaping. ROS 2, PX4, Gazebo and KR
are integration layers and must not enter the core API.

## Responsibility allocation

| Component | Owns | Must not own |
|---|---|---|
| Reference provider | Loading and source-level validation of trajectory data | MPC state propagation or PX4 mode changes |
| `MpcTrajectoryCore` | Virtual state, solver invocation, constraints, command/prediction validation and diagnostics | ROS topics, PX4 messages, arming, failsafe selection, attitude/rate or actuator control |
| Solver backend | Validated conversion to the selected solver API and solver status mapping | Frame conversion, vehicle estimation or runtime fallback policy |
| ROS 2 wrapper | Timestamp policy, lifecycle, timeout, parameters, logging and publication | Control mathematics |
| PX4 adapter | Explicit frame conversion and `TrajectorySetpoint` field mapping | Position feedback control or arming |
| PX4 | Position/velocity feedback, acceleration feedforward handling, attitude/rate/allocation and failsafe behavior | Owning the MPC virtual state |

## Control contract

The core operates on one declared frame and units:

```text
state/reference: position [m], velocity [m/s], acceleration [m/s^2]
time:           seconds, strictly increasing reference timestamps
output:         position, velocity, acceleration, yaw and yaw rate
```

The selected MRS solver has state `[position, velocity, acceleration]` and
the characterized dynamics:

```text
p[k+1] = p[k] + dt[k] * v[k]
v[k+1] = v[k] + dt[k] * a[k]
a[k+1] = p1 * a[k] + p2 * u[k]
```

Therefore raw `u[k]` is not a universal acceleration command. For horizontal
parameters `p1=0,p2=1`, `u[k]` equals the next acceleration state. For the
current vertical parameters `p1=0.5,p2=0.5`, the command boundary must use the
predicted acceleration state `a[k+1]`, not relabel raw `u[k]` as `a_d`.

## Virtual-state lifecycle

```text
UNCONFIGURED -> INACTIVE -> PRESTREAM -> ACTIVE
                                      |\
                                      | \-- HOLD
                                      | \-- FALLBACK
                                      \---- ERROR
```

- Activation initializes virtual position and velocity from measured state.
- Normal updates propagate virtual state internally.
- Measured state monitors tracking error and estimator validity.
- Reset/re-anchor is explicit and recorded; it is not an implicit per-cycle
  overwrite.
- Core reports validity and failure reason. Runtime layers choose pause, hold,
  fallback or PX4 mode behavior.

## Setpoint ownership

Only one component may own the active PX4 trajectory setpoint stream. The first
SITL integration must run no-arm message tests before any activation capable of
arming. Heartbeat publication is independent of the MPC solve timer.

## Target matrix

The following versions were observed locally on 2026-08-05. They are candidate
integration targets, not yet a compatibility proof:

| Item | Candidate | Status |
|---|---|---|
| PX4 Autopilot | `56dbe544fdedd0e69eb67f82c193137641310ccf`, `v1.17.0-44-g56dbe544fd` | observed, must be pinned in M0 |
| PX4 model | Gazebo `x500` / PX4 airframe `4001_gz_x500` | observed, must be verified in M7 |
| ROS 2 | Jazzy (`/opt/ros/jazzy`) | observed, must be recorded in toolchain manifest |
| Gazebo Sim | `8.11.0` | observed, must be recorded in toolchain manifest |
| `px4_msgs` | local tag `v1.17.0`, commit `86d8239e962f6939e05c3737784f60c02fa884db` | candidate only; compatibility with PX4 commit is unverified |
| MRS solver | `f173ea285cce7bbbfc1dbc382e31c761f44b0963` | observed and used by current contract tests |

M0 must replace the candidate entries with a reproducible manifest and license
audit. No SITL or hardware claim is implied by this table.

