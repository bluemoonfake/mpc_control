# Frame and time conventions

Status: M-1 baseline. No frame conversion is performed inside the pure MPC
core.

## Core convention

The core accepts a declared, common frame for measured state, reference,
virtual state and command. The frame declaration is an integration contract;
the core treats the three axes as ordered numeric axes and performs no hidden
ENU/NED conversion.

Required units:

| Quantity | Unit |
|---|---|
| position | m |
| velocity | m/s |
| acceleration | m/s^2 |
| yaw | rad, wrapped to `[-pi, pi)` |
| yaw rate | rad/s |
| core time | s |

Finite values and dimension checks occur before solver invocation.

## PX4 boundary convention

PX4 local position is NED:

```text
X_NED = North
Y_NED = East
Z_NED = Down
```

For a source trajectory expressed in standard ROS ENU coordinates:

```text
p_NED = [ p_ENU.y,  p_ENU.x, -p_ENU.z ]
v_NED = [ v_ENU.y,  v_ENU.x, -v_ENU.z ]
a_NED = [ a_ENU.y,  a_ENU.x, -a_ENU.z ]
```

The mapping is applied exactly once in the dedicated PX4 adapter. A source
that is already NED must be marked NED and must not pass through this mapping.

For the standard ENU yaw convention (angle from East, positive
counter-clockwise) and PX4 NED yaw convention (angle from North, positive
clockwise):

```text
yaw_NED = wrap_pi(pi/2 - yaw_ENU)
yawspeed_NED = -yawspeed_ENU
```

These formulas are policy, not yet executable adapter evidence. They require
signed-axis, yaw-wrap and no-double-conversion tests before M6.

## Time policy

- Reference timestamps are relative seconds from trajectory start and must be
  strictly increasing.
- Runtime update time is monotonic within one clock domain.
- A reference sample at update time `t_k` is evaluated at the exact solver
  prediction times `t_k + tau_i`; input rows are not assumed to equal solver
  steps.
- The current MRS solver characterization uses 26 prediction states. Its
  first step uses `dt1`; subsequent steps use `dt2`. The exact complete grid
  must be proven by M1 tests before a sampler is implemented.
- After the final reference point, the sampler emits an explicit hold at the
  final position with zero velocity and acceleration. Undefined extrapolation
  is forbidden.
- PX4 message timestamps use the PX4-required microsecond representation and
  are generated only by the adapter. Core timestamps remain in seconds.
- Out-of-order, paused-clock or stale data causes a deterministic invalid or
  hold transition according to the runtime state machine; old commands are not
  replayed indefinitely.

## Required verification

The adapter test suite must prove:

1. `+X_ENU -> +Y_NED`;
2. `+Y_ENU -> +X_NED`;
3. `+Z_ENU -> -Z_NED`;
4. velocity and acceleration use the same mapping;
5. yaw and yaw-rate signs are correct;
6. wrapping at `+-pi` is deterministic;
7. an already-NED trajectory is not converted twice;
8. timestamps remain monotonic and use the documented units.

