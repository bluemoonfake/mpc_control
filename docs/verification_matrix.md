# Requirements verification matrix

Status: M-1 baseline. Test IDs are stable names for planned evidence; a test
is not considered passed until its command and result artifact exist.

| Requirement group | Requirement IDs | Verification IDs | Current evidence | Gate |
|---|---|---|---|---|
| Reference loading/validation | REQ-FUNC-001, REQ-REF-001 | TEST-REF-VALIDATION, TEST-REF-MALFORMED | Not implemented | M2 |
| End-of-trajectory hold | REQ-REF-002 | TEST-FINAL-HOLD | Policy documented; sampler not implemented | M2/M4 |
| Exact prediction-grid sampling | REQ-FUNC-002 | TEST-SAMPLER-GRID | Solver source/contract partial; full grid unproven | M1/M2 |
| p/v/a/yaw output | REQ-FUNC-003 | TEST-CORE-OUTPUT | Core not implemented | M3 |
| Analytic fixtures | REQ-FUNC-004 | TEST-FIXTURE-CONSISTENCY | Not implemented | M2/M4 |
| Virtual activation | REQ-FUNC-005 | TEST-VIRTUAL-ACTIVATE | Core not implemented | M3 |
| Virtual propagation | REQ-FUNC-006 | TEST-VIRTUAL-PROPAGATION | Core not implemented | M3 |
| Tracking monitor | REQ-FUNC-007 | TEST-TRACKING-MONITOR | Policy documented; runtime not implemented | M3/M7 |
| Reset/re-anchor policy | REQ-FUNC-008 | TEST-RESET-POLICY | Core not implemented | M3 |
| PX4 feedback boundary | REQ-BOUNDARY-001 | TEST-ARCH-REVIEW | Architecture documented; runtime not implemented | M1/M3 |
| Solver command boundary | REQ-BOUNDARY-002 | TEST-SOLVER-SEMANTICS, TEST-PX4-SETPOINT | Solver contract partial; adapter not implemented | M1/M6 |
| Setpoint ownership | REQ-BOUNDARY-003 | TEST-SETPOINT-OWNERSHIP | Runtime not implemented | M6/M7 |
| Core isolation | REQ-BOUNDARY-004 | TEST-CORE-NO-ROS | Core not implemented | M3 |
| Core frame policy | REQ-FRAME-001 | TEST-FRAME-CORE | Policy documented; core not implemented | M2/M3 |
| ENU/NED adapter | REQ-FRAME-002 | TEST-ENU-NED | Policy documented; adapter not implemented | M6 |
| MPC timing | REQ-TIME-001 | TEST-TIMING-50HZ | Candidate 50 Hz policy documented | M4 |
| Heartbeat independence | REQ-TIME-002 | TEST-HEARTBEAT-INDEPENDENCE | Policy documented; runtime not implemented | M5/M7 |
| Stale/out-of-order data | REQ-TIME-003 | TEST-STALE-DATA | Policy documented; runtime not implemented | M2/M5/M8 |
| Solver semantics | REQ-BOUNDARY-002 | TEST-SOLVER-SEMANTICS | 7 MRS contract tests pass; failure/status boundary incomplete | M1 |
| Non-finite boundary | REQ-SAFE-002 | TEST-NONFINITE-BOUNDARY | Not implemented | M3/M6 |
| Inactive/error output policy | REQ-SAFE-003 | TEST-LIFECYCLE-OUTPUT | Not implemented | M5 |
| Solver failure policy | REQ-SAFE-004 | TEST-SOLVER-FAILURE | Not implemented | M1/M3/M8 |
| No-arm and setpoint ownership | REQ-BOUNDARY-003, REQ-SAFE-001 | TEST-SETPOINT-OWNERSHIP, TEST-NO-ARM | Not implemented | M6 |
| Estimator reset | REQ-SAFE-005 | TEST-ESTIMATOR-RESET | Policy documented; runtime not implemented | M3/M7/M8 |
| Tracking abort | REQ-SAFE-006 | TEST-TRACKING-ABORT | Policy documented; runtime not implemented | M3/M7/M8 |
| PX4 failsafe | REQ-SAFE-007 | TEST-OFFBOARD-FAILSAFE | PX4 config not pinned | M7/M8 |

## Evidence rules

- A unit or contract test records command, commit, configuration and result.
- SITL evidence records PX4 commit, `px4_msgs` commit, Gazebo version, model,
  launch command, parameter hash, seed, ROS bag and PX4 ULog.
- Metrics always separate shaping error `p_ref-p_d`, tracking error
  `p_d-p_uav` and total error `p_ref-p_uav`.
- A missing artifact is `NOT VERIFIED`, never `PASS`.
