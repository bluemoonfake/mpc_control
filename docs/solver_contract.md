# MRS MPC controller solver contract

Characterization target:

- repository: `https://github.com/ctu-mrs/mrs_mpc_solvers.git`
- branch: `ros2`
- commit: `f173ea285cce7bbbfc1dbc382e31c761f44b0963`
- horizon: 26 states and 26 control inputs

## State and dynamics

For one axis, the state is `[position, velocity, acceleration]`. Source
inspection and executable tests establish the first-step dynamics as:

```text
position[1]     = position[0] + dt1 * velocity[0]
velocity[1]     = velocity[0] + dt1 * acceleration[0]
acceleration[1] = p1 * acceleration[0] + p2 * control[0]
```

The same form uses `dt2` for subsequent prediction steps. The MRS controller
configures horizontal axes with `p1=0, p2=1`, and the vertical axis with
`p1=0.5, p2=0.5`.

## Confirmed behavior

The executable contract tests confirm:

- zero state and zero reference produce approximately zero control;
- positive and negative position steps produce sign-symmetric control;
- positive velocity error produces braking control;
- first control respects magnitude and first-step rate constraints;
- the first predicted horizontal and vertical states match the configured
  dynamics;
- nominal cases finish before the configured maximum iteration count;
- all tested outputs and predictions are finite.

## Unresolved backend limitations

- `solveMPC()` does not expose an unambiguous convergence flag, primal/dual
  residuals, or objective value through the public wrapper.
- Public methods do not validate state/reference/weight dimensions or finite
  values before indexing them. These checks must live in the new backend.
- The generated horizon is fixed at 26.
- The current `setDt()` implementation must not be used until separately
  corrected and regression-tested; construct a solver with the desired time
  parameters instead.
- Invalid-dimension and non-finite-input tests belong at the safe
  `MrsSolverBackend` boundary, not directly against this unchecked API.
