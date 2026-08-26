"""Symbolic TMPC model and acados OCP configuration.

This module is a code-generation boundary.  The runtime controller must not
import CasADi or acados_template; it consumes the generated C interface.
"""

from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import ACADOS_INFTY, AcadosModel, AcadosOcp


PHYSICAL_STATE_DIMENSION = 11
COMMAND_MEMORY_DIMENSION = 4
STATE_DIMENSION = PHYSICAL_STATE_DIMENSION + COMMAND_MEMORY_DIMENSION
INPUT_DIMENSION = 4
STATE_COST_DIMENSION = PHYSICAL_STATE_DIMENSION + 1
INPUT_COST_DIMENSION = INPUT_DIMENSION + 3
STAGE_COST_DIMENSION = STATE_COST_DIMENSION + INPUT_COST_DIMENSION
PARAMETER_DIMENSION = 6
HORIZON_LENGTH = 10
SAMPLE_TIME_SECONDS = 0.05
MAX_TILT_RADIANS = 0.7853981633974483
MIN_COLLECTIVE_SPECIFIC_FORCE_MPS2 = 7.0
MAX_COLLECTIVE_SPECIFIC_FORCE_MPS2 = 14.0

DEFAULT_GRAVITY = 9.80665
DEFAULT_MODEL_PARAMETERS = np.array(
    [0.18, 0.18, 3.42, 0.102, 0.0932, DEFAULT_GRAVITY], dtype=float
)

DEFAULT_STAGE_WEIGHTS = np.array(
    [20.0, 20.0, 80.0, 25.0, 25.0, 60.0, 5.0, 5.0, 20.0, 15.0, 15.0],
    dtype=float,
)
DEFAULT_TERMINAL_WEIGHTS = np.array(
    [30.0, 30.0, 100.0, 30.0, 30.0, 70.0, 10.0, 10.0, 25.0, 20.0, 20.0],
    dtype=float,
)
DEFAULT_INPUT_WEIGHTS = np.array([80.0, 80.0, 15.0, 20.0], dtype=float)
DEFAULT_YAW_COMMAND_DELTA_WEIGHT = 25.0

DEFAULT_STATE_LOWER = np.full(STATE_DIMENSION, -1.0e6, dtype=float)
DEFAULT_STATE_UPPER = np.full(STATE_DIMENSION, 1.0e6, dtype=float)
DEFAULT_INPUT_LOWER = np.full(INPUT_DIMENSION, -1.0e6, dtype=float)
DEFAULT_INPUT_UPPER = np.full(INPUT_DIMENSION, 1.0e6, dtype=float)
MAX_YAW_COMMAND_RADIANS = 1.0e6

DEFAULT_STATE_LOWER[6:8] = -MAX_TILT_RADIANS
DEFAULT_STATE_UPPER[6:8] = MAX_TILT_RADIANS
DEFAULT_STATE_LOWER[9] = -2.0
DEFAULT_STATE_UPPER[9] = 2.0
DEFAULT_STATE_LOWER[10] = MIN_COLLECTIVE_SPECIFIC_FORCE_MPS2
DEFAULT_STATE_UPPER[10] = MAX_COLLECTIVE_SPECIFIC_FORCE_MPS2
DEFAULT_INPUT_LOWER[0:2] = -MAX_TILT_RADIANS
DEFAULT_INPUT_UPPER[0:2] = MAX_TILT_RADIANS
DEFAULT_INPUT_LOWER[2] = -MAX_YAW_COMMAND_RADIANS
DEFAULT_INPUT_UPPER[2] = MAX_YAW_COMMAND_RADIANS
DEFAULT_INPUT_LOWER[3] = MIN_COLLECTIVE_SPECIFIC_FORCE_MPS2
DEFAULT_INPUT_UPPER[3] = MAX_COLLECTIVE_SPECIFIC_FORCE_MPS2
DEFAULT_STATE_LOWER[11:15] = DEFAULT_INPUT_LOWER
DEFAULT_STATE_UPPER[11:15] = DEFAULT_INPUT_UPPER


def state_cost_expression(state: ca.SX) -> ca.SX:
    """Return a periodic yaw residual while preserving all other states."""

    return ca.vertcat(
        state[0:8],
        state[9:11],
        ca.sin(state[8]),
        ca.cos(state[8]),
    )


def input_cost_expression(state: ca.SX, control: ca.SX) -> ca.SX:
    """Return separate periodic yaw target and yaw-command-delta residuals."""

    return ca.vertcat(
        control[0:2],
        control[3],
        ca.sin(control[2]),
        ca.cos(control[2]),
        ca.sin(control[2] - state[13]),
        ca.cos(control[2] - state[13]),
    )


def state_cost_weights(weights: np.ndarray) -> np.ndarray:
    """Map state weights to the residual that represents yaw by sin/cos."""

    return np.concatenate((weights[0:8], weights[9:11], [weights[8], weights[8]]))


def input_cost_weights(weights: np.ndarray) -> np.ndarray:
    """Map input and delta-yaw weights to the periodic residuals."""

    return np.concatenate(
        (
            weights[0:2],
            weights[3:4],
            [weights[2], weights[2]],
            [DEFAULT_YAW_COMMAND_DELTA_WEIGHT, DEFAULT_YAW_COMMAND_DELTA_WEIGHT],
        )
    )


def create_tpmc_model() -> AcadosModel:
    """Create the symbolic nonlinear TMPC dynamics model."""

    state = ca.SX.sym("x", STATE_DIMENSION)
    control = ca.SX.sym("u", INPUT_DIMENSION)
    parameters = ca.SX.sym("p", PARAMETER_DIMENSION)

    physical_state = state[0:PHYSICAL_STATE_DIMENSION]
    position = physical_state[0:3]
    velocity = physical_state[3:6]
    roll = physical_state[6]
    pitch = physical_state[7]
    yaw = physical_state[8]
    yaw_rate = physical_state[9]
    collective_specific_force = physical_state[10]
    previous_command = state[PHYSICAL_STATE_DIMENSION:STATE_DIMENSION]

    roll_command = control[0]
    pitch_command = control[1]
    yaw_command = control[2]
    collective_specific_force_command = control[3]

    roll_time_constant = parameters[0]
    pitch_time_constant = parameters[1]
    yaw_natural_frequency = parameters[2]
    yaw_damping_ratio = parameters[3]
    force_time_constant = parameters[4]
    gravity = parameters[5]

    sin_roll = ca.sin(roll)
    cos_roll = ca.cos(roll)
    sin_pitch = ca.sin(pitch)
    cos_pitch = ca.cos(pitch)
    sin_yaw = ca.sin(yaw)
    cos_yaw = ca.cos(yaw)

    body_z = ca.vertcat(
        cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll,
        sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll,
        cos_pitch * cos_roll,
    )

    position_derivative = velocity
    velocity_derivative = ca.vertcat(
        body_z[0] * collective_specific_force,
        body_z[1] * collective_specific_force,
        body_z[2] * collective_specific_force - gravity,
    )
    wrapped_yaw_command_error = ca.atan2(
        ca.sin(yaw_command - yaw), ca.cos(yaw_command - yaw)
    )
    roll_pitch_derivative = ca.vertcat(
        (roll_command - roll) / roll_time_constant,
        (pitch_command - pitch) / pitch_time_constant,
    )
    yaw_derivative = yaw_rate
    yaw_rate_derivative = (
        yaw_natural_frequency**2 * wrapped_yaw_command_error
        - 2.0 * yaw_damping_ratio * yaw_natural_frequency * yaw_rate
    )
    force_derivative = (
        collective_specific_force_command - collective_specific_force
    ) / force_time_constant

    model = AcadosModel()
    model.name = "tpmc"
    model.x = state
    model.u = control
    model.p = parameters
    physical_derivative = ca.vertcat(
        position_derivative,
        velocity_derivative,
        roll_pitch_derivative,
        yaw_derivative,
        yaw_rate_derivative,
        force_derivative,
    )
    physical_dynamics = ca.Function(
        "tpmc_physical_dynamics", [physical_state, control, parameters],
        [physical_derivative],
    )
    first_derivative = physical_dynamics(physical_state, control, parameters)
    second_derivative = physical_dynamics(
        physical_state + 0.5 * SAMPLE_TIME_SECONDS * first_derivative,
        control,
        parameters,
    )
    third_derivative = physical_dynamics(
        physical_state + 0.5 * SAMPLE_TIME_SECONDS * second_derivative,
        control,
        parameters,
    )
    fourth_derivative = physical_dynamics(
        physical_state + SAMPLE_TIME_SECONDS * third_derivative,
        control,
        parameters,
    )
    physical_next = physical_state + SAMPLE_TIME_SECONDS * (
        first_derivative + 2.0 * second_derivative + 2.0 * third_derivative + fourth_derivative
    ) / 6.0
    model.disc_dyn_expr = ca.vertcat(
        physical_next,
        control,
    )
    model.x_labels = [
        "position_x",
        "position_y",
        "position_z",
        "velocity_x",
        "velocity_y",
        "velocity_z",
        "roll",
        "pitch",
        "yaw",
        "yaw_rate",
        "collective_specific_force",
        "previous_roll_command",
        "previous_pitch_command",
        "previous_yaw_command",
        "previous_collective_specific_force_command",
    ]
    model.u_labels = [
        "roll_command",
        "pitch_command",
        "yaw_command",
        "collective_specific_force_command",
    ]
    model.p_labels = [
        "roll_time_constant",
        "pitch_time_constant",
        "yaw_natural_frequency",
        "yaw_damping_ratio",
        "force_time_constant",
        "gravity",
    ]

    # The tilt limit is expressed as body_z_z >= cos(max_tilt).  This is
    # equivalent to tilt <= max_tilt for the configured max_tilt < pi/2 and
    # avoids the derivative singularity of acos at the constraint boundary.
    model.con_h_expr = ca.vertcat(body_z[2], control - previous_command)
    model.con_h_expr_e = body_z[2]
    return model


def create_tpmc_ocp(output_directory: Path) -> AcadosOcp:
    """Create the nonlinear least-squares OCP consumed by acados_template."""

    model = create_tpmc_model()
    ocp = AcadosOcp()
    ocp.model = model
    ocp.code_gen_options.code_export_directory = str(output_directory)
    ocp.code_gen_options.json_file = str(output_directory / "tpmc_ocp.json")

    ocp.solver_options.N_horizon = HORIZON_LENGTH
    ocp.solver_options.tf = HORIZON_LENGTH * SAMPLE_TIME_SECONDS
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = HORIZON_LENGTH // 2
    ocp.solver_options.hessian_approx = "EXACT"
    # RTI performs one linearization and one QP feedback step per 50 Hz
    # control cycle. The shifted previous solution remains the warm start.
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    # Retain QP residuals without enabling RTI KKT residual recomputation,
    # which adds a separate nonlinear-function evaluation to every cycle.
    ocp.solver_options.nlp_solver_ext_qp_res = 1
    # acados supports funnel line search only for multi-iterate SQP. RTI uses
    # its required deterministic full feedback step instead.
    ocp.solver_options.globalization = "FIXED_STEP"
    ocp.solver_options.globalization_fixed_step_length = 1.0
    # The tilt envelope is a nonlinear BGH constraint. acados explicitly
    # disallows CONVEXIFY with nonlinear constraints, so use MIRROR to keep
    # the exact-Hessian QP well conditioned without invalidating that
    # constraint linearization.
    ocp.solver_options.regularize_method = "MIRROR"
    ocp.solver_options.reg_epsilon = 1.0e-5
    ocp.solver_options.levenberg_marquardt = 1.0e-3
    # ROBUST uses HPIPM's numerically conservative barrier settings.  This is
    # important for the hard tilt/force bounds when External Mode is entered
    # from a state that is not on the previous MPC prediction.
    ocp.solver_options.hpipm_mode = "ROBUST"
    # Keep QP termination deterministic instead of relying on the generated
    # default.
    ocp.solver_options.qp_solver_tol_stat = 1.0e-4
    ocp.solver_options.qp_solver_tol_eq = 1.0e-4
    ocp.solver_options.qp_solver_tol_ineq = 1.0e-4
    ocp.solver_options.qp_solver_tol_comp = 1.0e-4
    ocp.solver_options.print_level = 0

    ocp.model.cost_y_expr = ca.vertcat(
        state_cost_expression(model.x), input_cost_expression(model.x, model.u)
    )
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.W = np.diag(
        np.concatenate(
            (state_cost_weights(DEFAULT_STAGE_WEIGHTS),
             input_cost_weights(DEFAULT_INPUT_WEIGHTS))
        )
    )
    ocp.cost.yref = np.zeros(STAGE_COST_DIMENSION, dtype=float)

    ocp.model.cost_y_expr_e = state_cost_expression(model.x)
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W_e = np.diag(state_cost_weights(DEFAULT_TERMINAL_WEIGHTS))
    ocp.cost.yref_e = np.zeros(STATE_COST_DIMENSION, dtype=float)

    state_indices = np.arange(STATE_DIMENSION, dtype=int)
    input_indices = np.arange(INPUT_DIMENSION, dtype=int)
    ocp.constraints.x0 = np.zeros(STATE_DIMENSION, dtype=float)
    ocp.constraints.idxbx = state_indices
    ocp.constraints.lbx = DEFAULT_STATE_LOWER
    ocp.constraints.ubx = DEFAULT_STATE_UPPER
    ocp.constraints.idxbu = input_indices
    ocp.constraints.lbu = DEFAULT_INPUT_LOWER
    ocp.constraints.ubu = DEFAULT_INPUT_UPPER

    maximum_tilt_command_change = 2.0 * SAMPLE_TIME_SECONDS
    maximum_yaw_command_change = 2.0 * SAMPLE_TIME_SECONDS
    maximum_collective_change = 25.0 * SAMPLE_TIME_SECONDS
    ocp.constraints.lh = np.array(
        [np.cos(MAX_TILT_RADIANS), -maximum_tilt_command_change,
         -maximum_tilt_command_change, -maximum_yaw_command_change,
         -maximum_collective_change], dtype=float)
    ocp.constraints.uh = np.array(
        [ACADOS_INFTY, maximum_tilt_command_change,
         maximum_tilt_command_change, maximum_yaw_command_change,
         maximum_collective_change], dtype=float)
    ocp.constraints.lh_e = np.array([np.cos(MAX_TILT_RADIANS)], dtype=float)
    ocp.constraints.uh_e = np.array([ACADOS_INFTY], dtype=float)
    ocp.parameter_values = DEFAULT_MODEL_PARAMETERS
    return ocp
