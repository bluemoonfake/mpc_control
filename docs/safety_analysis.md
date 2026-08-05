# Initial safety analysis

Status: M-1 lightweight hazard analysis. This is a development safety
baseline, not a flight-approval document.

## Safety invariants

1. No automatic arming is present in the core or no-arm integration tests.
2. No non-finite value reaches the PX4 message boundary.
3. Only one active trajectory setpoint owner exists.
4. A stale reference or vehicle state cannot replay an old command forever.
5. Solver failure, timing failure and estimator reset have explicit states.
6. Virtual state is not silently overwritten by measured state every cycle.
7. PX4 safety checks and failsafe behavior are not disabled to make a test pass.
8. Hardware flight is out of scope until no-arm, SITL nominal and SITL fault
   gates pass.

## Hazard table

| ID | Hazard | Cause | Effect | Detection | Mitigation | Verification |
|---|---|---|---|---|---|---|
| HAZ-001 | Z direction reversed | ENU/NED sign error | Uncommanded climb/descent | Signed-axis adapter test | One conversion module, NED boundary contract | TEST-ENU-NED |
| HAZ-002 | Raw solver input mislabeled as acceleration | Ignoring `p1/p2` dynamics | Wrong feedforward and unstable shaping | Solver semantic review/golden test | Map predicted `a[k+1]` to `a_d` | TEST-SOLVER-SEMANTICS |
| HAZ-003 | Virtual trajectory runs ahead | PX4 tracking divergence | Large catch-up command | `p_d - p_uav` monitor | Pause/hold/fallback thresholds | TEST-TRACKING-ABORT |
| HAZ-004 | Dual setpoint owners | KR or another bridge publishes concurrently | Undefined active command | ROS graph/topic inspection | Single owner policy and activation check | TEST-SETPOINT-OWNERSHIP |
| HAZ-005 | Solver overrun | CPU load or bad conditioning | Jitter and stale setpoints | P99/max timing monitor | Independent heartbeat and deterministic fault state | TEST-TIMING-50HZ |
| HAZ-006 | Non-finite command | Invalid input or numerical failure | Invalid PX4 setpoint | `isfinite` validation | Reject before adapter/publication | TEST-NONFINITE-BOUNDARY |
| HAZ-007 | Stale reference | Provider stops or clock pauses | Replay of old trajectory | Timestamp age/out-of-order check | Explicit hold/fallback; no indefinite replay | TEST-STALE-DATA |
| HAZ-008 | Stale vehicle state | Estimator/DDS interruption | Controller acts on obsolete state | State age monitor | Stop progression and enter documented state | TEST-STATE-STALE |
| HAZ-009 | Estimator reset ignored | Local frame/state discontinuity | Command discontinuity | PX4 reset counter/diagnostic | Reset/re-anchor policy and continuity check | TEST-ESTIMATOR-RESET |
| HAZ-010 | Unexpected arming | Test path sends arm command | Unsafe activation | Command topic/bag inspection | No-arm tests and code review | TEST-NO-ARM |
| HAZ-011 | Failure reuses old command forever | Solver/backend failure | Persistent stale command | Failure counter and command age | Fail closed; explicit hold/fallback | TEST-SOLVER-FAILURE |
| HAZ-012 | Final trajectory extrapolation | Missing terminal policy | Undefined future command | End-of-horizon test | Final hold `[p_final,0,0]` | TEST-FINAL-HOLD |

## Runtime response policy

The core returns validity and diagnostics. The wrapper/runtime owns transitions:

```text
ACTIVE -> PAUSED   warning tracking error or reference pause
ACTIVE -> HOLD     stale/recoverable fault or bounded tracking error
ACTIVE -> FALLBACK repeated solver/timing/state failure
any    -> ERROR    non-finite/configuration/internal fatal fault
```

Exact thresholds, PX4 fallback mode and failsafe parameters are not selected in
M-1. They must be configured, reviewed and tested before SITL activation.

## Residual risks and exclusions

This document does not approve real flight, automatic takeoff/landing,
geofence configuration, manual override behavior, HIL or KR active ownership.
Those require separate test cards and safety review.

