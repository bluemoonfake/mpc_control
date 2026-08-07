from mpc_control_msgs.msg import (
    MpcDiagnostics,
    PredictedTrajectory,
    ReferenceTrajectory,
    TrajectoryCommand,
    TrajectoryPoint,
    VehicleState,
)


def test_trajectory_point_schema():
    message = TrajectoryPoint()
    assert len(message.position) == 3
    assert len(message.velocity) == 3
    assert len(message.acceleration) == 3
    assert message.yaw_valid is False


def test_reference_and_prediction_contain_points():
    reference = ReferenceTrajectory()
    prediction = PredictedTrajectory()
    assert reference.points == []
    assert prediction.points == []
    assert reference.hold_after_end is False


def test_command_and_vehicle_state_schema():
    command = TrajectoryCommand()
    state = VehicleState()
    assert len(command.position) == 3
    assert len(command.velocity) == 3
    assert len(command.acceleration) == 3
    assert len(state.position) == 3
    assert len(state.velocity) == 3
    assert len(state.acceleration) == 3
    assert state.valid is False


def test_diagnostics_constants_and_arrays():
    diagnostics = MpcDiagnostics()
    assert diagnostics.LIFECYCLE_UNCONFIGURED == 0
    assert diagnostics.LIFECYCLE_INACTIVE == 1
    assert diagnostics.LIFECYCLE_ACTIVE == 2
    assert diagnostics.LIFECYCLE_ERROR == 3
    assert diagnostics.FAILURE_SOLVER_X == 8
    assert diagnostics.FAILURE_SOLVER_Y == 9
    assert diagnostics.FAILURE_SOLVER_Z == 10
    assert len(diagnostics.solver_iterations) == 3
    assert len(diagnostics.tracking_error_position) == 3
