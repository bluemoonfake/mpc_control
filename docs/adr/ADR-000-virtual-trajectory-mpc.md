# ADR-000: Use MPC as a virtual trajectory generator

- Status: Accepted for the first implementation
- Date: 2026-08-05
- Scope: standalone core through initial PX4 `gz_x500` SITL

## Context

The project needs to reshape a known reference trajectory before PX4 tracks it.
The first implementation must remain testable without ROS 2, PX4, Gazebo or KR
and must avoid introducing a second position-control owner or a custom
low-level attitude controller.

The existing MRS solver exposes a one-axis state `[p,v,a]` and a solver input
whose physical meaning depends on model parameters. Treating that raw input as
the PX4 acceleration field would be incorrect for the current filtered
vertical model.

## Decision

Use a pure C++ MPC virtual trajectory generator with internal virtual state
`[p_m,v_m,a_m]`.

The data flow is:

```text
validated p_ref,v_ref,a_ref,yaw_ref
  -> MpcTrajectoryCore
  -> validated p_d,v_d,a_d,yaw_d,yaw_rate_d
  -> PX4 TrajectorySetpoint
```

PX4 remains the sole owner of position/velocity feedback, acceleration/thrust
conversion, attitude, rate, allocation and failsafe behavior. The core does not
estimate state, publish ROS/PX4 messages, arm, select flight modes or command
actuators.

The adapter exposes the predicted state acceleration as `a_d`; it never assumes
that raw solver `u` is universally acceleration. Measured vehicle state is used
for activation and monitoring, not as a hidden per-cycle virtual-state reset.

## Alternatives rejected

1. **Acceleration-only MPC output** — rejected because it changes the PX4
   control boundary and bypasses the required position/velocity setpoint path.
2. **External PID or custom SO(3) controller** — rejected because it creates
   duplicate control ownership and is outside the first validation scope.
3. **MPC as estimator** — rejected because estimation and trajectory shaping
   have different state, timing and failure semantics.
4. **Direct motor/thrust output** — rejected because it bypasses PX4 safety and
   lower-level control allocation.
5. **Measured-state overwrite every cycle** — rejected because it hides
   tracking divergence and destroys the intended virtual-state dynamics.

## Consequences

Positive:

- The core can be unit-tested and model-tested without middleware.
- PX4's existing stabilization and failsafe stack remains in the loop.
- Shaping error and tracking error can be measured separately.
- The solver backend can be characterized and replaced independently.

Costs and follow-up:

- A safe solver backend must validate dimensions/finiteness and map ambiguous
  solver status before production use.
- ENU/NED conversion requires a dedicated adapter and signed-axis tests.
- The virtual-state pause/hold/fallback policy must be implemented before
  SITL activation.
- Dependency versions and exact `px4_msgs` compatibility remain M0 work.

## Reversal criteria

Reconsider this decision only after evidence shows that PX4's internal
trajectory tracking is the limiting factor across the approved operating
envelope, and after a separate safety review. Gain tuning must not be used to
mask frame, solver semantics, timing or ownership errors.

