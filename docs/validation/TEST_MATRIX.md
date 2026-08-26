# Test Matrix & Automated Test Pyramid (TEST_MATRIX.md)

This document catalogs the complete test pyramid across Unit, Integration, Solver, SITL, and Stress test layers.

---

## 1. Test Pyramid Architecture

```text
               ▲
              / \
             /   \      Level 5: Robustness & Stress Tests
            /     \     (cpu_load.py stress, multi-run repeats)
           /───────\
          /         \     Level 4: SITL Mission Simulation Tests
         /           \    (make sim -> make mission-execute -> validation-report)
        /─────────────\
       /               \    Level 3: Python Validation & Analysis Tests
      /                 \   (test/python/analyze_validation_run_test.py)
     /───────────────────\
    /                     \   Level 1 & 2: C++ Core Math & Solver Tests
   /                       \  (test/cpp/tpmc_core_test.cpp)
  /─────────────────────────\
```

---

## 2. Test Catalog and Coverage Details

### Level 1 & 2: C++ Core Unit Tests (`test/cpp/tpmc_core_test.cpp`)

| Test Suite / Case | Code Symbols Exercised | Verified Invariants & Assertions | Execution Time | Command |
| :--- | :--- | :--- | :---: | :--- |
| `ERK4 Dynamics Discretization` | `tpmc::createModel()`, `model.continuousDynamics()` | Free-fall under gravity accelerates at $-9.80665\text{ m/s}^2$; zero net acceleration when $a = g$. | $< 5\text{ ms}$ | `colcon test` |
| `Tilt Cone & Box Constraints` | `tpmc::checkBounds()`, `tpmc::describePredictionViolation()` | BGH nonlinear constraint correctly flags body $z_z < \cos(45^\circ)$; verifies input rate bounds. | $< 2\text{ ms}$ | `colcon test` |
| `3D Corner Velocity Planning` | `blendedCornerVelocity()` | Correctly computes bisector direction $\vec{u}_{bisector}$ and scales speed by $\cos^3(\theta/2)$. | $< 2\text{ ms}$ | `colcon test` |
| `Command Safety Limiter` | `command_safety::Limiter` | Restricts excessive tilt angle, projects attitude onto allowable cone, clamps slew rates. | $< 5\text{ ms}$ | `colcon test` |
| `Geometric Fallback Controller`| `control::GeometricController` | Generates stable recovery specific force and attitude for step position errors. | $< 5\text{ ms}$ | `colcon test` |
| `Collective Force Filter` | `filter::CollectiveForceFilter` | Low-pass filters noisy specific force measurements; extracts steady-state hover force. | $< 2\text{ ms}$ | `colcon test` |
| `State Bridge Conversions` | `frame::convert()`, `nedToEnu()` | Verifies Hamilton quaternion sign conventions and Euler angle extraction consistency. | $< 5\text{ ms}$ | `colcon test` |
| `Acados Solver Cold/Warm Start`| `tpmc::AcadosTpmcSolver` | Solves nominal 10-step OCP in $< 2.0\text{ ms}$; confirms warm-start shifting reduces iterations. | $< 80\text{ ms}$ | `colcon test` |

---

### Level 3: Python Validation Engine Tests (`test/python/analyze_validation_run_test.py`)

| Test Case Name | Tested Function / Logic | Verification Purpose | Command |
| :--- | :--- | :--- | :--- |
| `test_motor_saturation_requires_fresh_measurements` | `validation.motor_saturation_stats` | Rejects stale motor samples older than $100\text{ ms}$. | `pytest test/` |
| `test_motor_saturation_detects_upper_limit` | `validation.motor_saturation_stats` | Correctly flags actuator saturation when $u_{max} \ge 0.90$. | `pytest test/` |
| `test_tracking_summary_handles_empty_samples` | `validation.tracking_summary` | Gracefully handles empty mission segments without division by zero. | `pytest test/` |
| `test_solve_time_percentiles_calculation` | `validation.solve_time_stats` | Validates p95, p99, and max solve time calculations against analytical values. | `pytest test/` |
| `test_deadline_miss_gate_evaluation` | `validation.evaluate_gates` | Flags run as `FAIL` if solve time exceeds $18.0\text{ ms}$ on $> 1\%$ samples. | `pytest test/` |
| `test_hover_stability_euler_std` | `validation.hover_stability_stats` | Verifies standard deviation formula for roll and pitch stability. | `pytest test/` |
