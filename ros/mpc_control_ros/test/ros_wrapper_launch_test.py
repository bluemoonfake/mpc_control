import os
import time
import unittest

from ament_index_python.packages import get_package_prefix

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import launch_testing.tools

import pytest
import rclpy
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rosgraph_msgs.msg import Clock

from builtin_interfaces.msg import Duration
from mpc_control_msgs.msg import (
    MpcDiagnostics,
    ReferenceTrajectory,
    TrajectoryCommand,
    TrajectoryPoint,
    VehicleState,
)


def _node_executable():
    return os.path.join(
        get_package_prefix("mpc_control_ros"),
        "lib",
        "mpc_control_ros",
        "mpc_trajectory_node",
    )


def _process_action(node_name, parameters=None):
    command = [
        _node_executable(),
        "--ros-args",
        "-r",
        "__node:=" + node_name,
    ]
    for name, value in (parameters or {}).items():
        command.extend(["-p", "{}:={}".format(name, value)])
    return launch.actions.ExecuteProcess(
        cmd=command,
        name=node_name,
        output="screen",
    )


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_testing.actions.ReadyToTest(),
    ])


class RosLifecycleHarness:
    def __init__(self, node, node_name):
        self.node = node
        self.node_name = node_name
        self.change_state = node.create_client(
            ChangeState, "/{}/change_state".format(node_name)
        )
        self.get_state = node.create_client(
            GetState, "/{}/get_state".format(node_name)
        )
        self.set_parameters = node.create_client(
            SetParameters, "/{}/set_parameters".format(node_name)
        )

    def destroy(self):
        self.node.destroy_client(self.change_state)
        self.node.destroy_client(self.get_state)
        self.node.destroy_client(self.set_parameters)

    def _call(self, client, request, timeout=5.0):
        self.assert_service(client, timeout)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout
        )
        if not future.done() or future.result() is None:
            raise AssertionError("service call did not complete")
        return future.result()

    @staticmethod
    def assert_service(client, timeout):
        if not client.wait_for_service(timeout_sec=timeout):
            raise AssertionError("lifecycle service did not appear")

    def state(self):
        return self._call(self.get_state, GetState.Request()).current_state.id

    def transition(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        return self._call(self.change_state, request).success

    def set_double_parameter(self, name, value):
        request = SetParameters.Request()
        request.parameters = [
            ParameterMessage(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=value,
                ),
            )
        ]
        response = self._call(self.set_parameters, request)
        if not response.results or not response.results[0].successful:
            raise AssertionError("parameter update failed: {}".format(name))


class TestRosWrapperLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(
            "ros_wrapper_launch_test_{}".format(os.getpid())
        )

    def tearDown(self):
        self.node.destroy_node()

    def _start_and_activate(self, launch_service, proc_info, proc_output,
                            node_name, parameters=None):
        action = _process_action(node_name, parameters)
        process = launch_testing.tools.launch_process(
            launch_service, action, proc_info, proc_output
        )
        process.__enter__()
        proc_info.assertWaitForStartup(process=action, timeout=5)
        harness = RosLifecycleHarness(self.node, node_name)
        self.assertEqual(harness.state(), State.PRIMARY_STATE_UNCONFIGURED)
        self.assertTrue(harness.transition(Transition.TRANSITION_CONFIGURE))
        self.assertEqual(harness.state(), State.PRIMARY_STATE_INACTIVE)
        self.assertTrue(harness.transition(Transition.TRANSITION_ACTIVATE))
        self.assertEqual(harness.state(), State.PRIMARY_STATE_ACTIVE)
        return process, action, harness

    @staticmethod
    def _spin_until(node, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return True
        return predicate()

    def _clock_message(self, seconds):
        message = Clock()
        message.clock.sec = int(seconds)
        message.clock.nanosec = int((seconds - int(seconds)) * 1.0e9)
        return message

    def _reference(self, seconds):
        message = ReferenceTrajectory()
        message.header.stamp.sec = int(seconds)
        message.header.stamp.nanosec = 0
        message.header.frame_id = "map"
        message.trajectory_id = 7
        message.hold_after_end = True
        for offset in (0, 10):
            point = TrajectoryPoint()
            point.time_from_start = Duration(sec=offset, nanosec=0)
            point.position = [0.0, 0.0, 0.0]
            point.velocity = [0.0, 0.0, 0.0]
            point.acceleration = [0.0, 0.0, 0.0]
            point.yaw_valid = False
            message.points.append(point)
        return message

    def _state(self, seconds):
        message = VehicleState()
        message.header.stamp.sec = int(seconds)
        message.header.stamp.nanosec = 0
        message.header.frame_id = "map"
        message.sequence = 1
        message.position = [0.0, 0.0, 0.0]
        message.velocity = [0.0, 0.0, 0.0]
        message.acceleration = [0.0, 0.0, 0.0]
        message.valid = True
        message.position_valid = True
        message.velocity_valid = True
        message.acceleration_valid = False
        message.yaw_valid = True
        message.reset_counter_valid = True
        message.reset_counter = 0
        return message

    def test_simulated_clock_pause_and_forward_jump(self, launch_service,
                                                     proc_info, proc_output):
        node_name = "mpc_ros_wp3_clock"
        process, action, harness = self._start_and_activate(
            launch_service,
            proc_info,
            proc_output,
            node_name,
            parameters={
                "use_sim_time": "true",
                "clock_stall_timeout_seconds": 0.15,
                "clock_jump_threshold_seconds": 0.5,
                "update_rate_hz": 20.0,
            },
        )
        try:
            clock_pub = self.node.create_publisher(Clock, "/clock", 10)
            reference_pub = self.node.create_publisher(
                ReferenceTrajectory, "reference_trajectory", 10
            )
            state_pub = self.node.create_publisher(VehicleState, "vehicle_state", 10)
            commands = []
            diagnostics = []
            command_sub = self.node.create_subscription(
                TrajectoryCommand,
                "trajectory_command",
                lambda message: commands.append(message),
                10,
            )
            diagnostics_sub = self.node.create_subscription(
                MpcDiagnostics,
                "mpc_diagnostics",
                lambda message: diagnostics.append(message),
                10,
            )
            self.assertTrue(
                self._spin_until(
                    self.node,
                    lambda: clock_pub.get_subscription_count() > 0
                    and reference_pub.get_subscription_count() > 0
                    and state_pub.get_subscription_count() > 0,
                )
            )

            clock_pub.publish(self._clock_message(1000.0))
            self._spin_until(self.node, lambda: True, timeout=0.1)
            reference = self._reference(1000.0)
            state = self._state(1000.0)
            for _ in range(8):
                reference_pub.publish(reference)
                state_pub.publish(state)
                rclpy.spin_once(self.node, timeout_sec=0.02)
            self.assertTrue(
                self._spin_until(self.node, lambda: len(commands) > 0, timeout=2.0)
            )

            paused_count = len(commands)
            self.assertTrue(
                self._spin_until(
                    self.node,
                    lambda: any(
                        message.failure_reason == MpcDiagnostics.FAILURE_TIME_JUMP
                        and "paused" in message.detail
                        for message in diagnostics
                    ),
                    timeout=2.0,
                )
            )
            failed_count = len(commands)
            self._spin_until(
                self.node,
                lambda: False,
                timeout=0.25,
            )
            self.assertLessEqual(failed_count, paused_count + 1)
            self.assertEqual(len(commands), failed_count)

            clock_pub.publish(self._clock_message(1001.0))
            self.assertTrue(
                self._spin_until(
                    self.node,
                    lambda: any(
                        message.header.stamp.sec == 1001
                        and "forward discontinuously" in message.detail
                        for message in diagnostics
                    ),
                    timeout=2.0,
                )
            )
            self.assertEqual(len(commands), failed_count)
            self.node.destroy_subscription(command_sub)
            self.node.destroy_subscription(diagnostics_sub)
        finally:
            process.__exit__(None, None, None)

    def test_process_restart_recreates_lifecycle_services(self, launch_service,
                                                           proc_info, proc_output):
        node_name = "mpc_ros_wp3_restart"
        process, action, harness = self._start_and_activate(
            launch_service, proc_info, proc_output, node_name
        )
        harness.destroy()
        process.__exit__(None, None, None)
        self._spin_until(self.node, lambda: False, timeout=0.5)

        restarted_process, restarted_action, restarted_harness = (
            self._start_and_activate(
                launch_service, proc_info, proc_output, node_name
            )
        )
        try:
            self.assertEqual(
                restarted_harness.state(), State.PRIMARY_STATE_ACTIVE
            )
        finally:
            restarted_process.__exit__(None, None, None)

    def test_failed_configuration_recovers_after_parameter_fix(self, launch_service,
                                                                proc_info, proc_output):
        node_name = "mpc_ros_wp3_error"
        action = _process_action(node_name, {"update_rate_hz": "-1.0"})
        process = launch_testing.tools.launch_process(
            launch_service, action, proc_info, proc_output
        )
        process.__enter__()
        try:
            proc_info.assertWaitForStartup(process=action, timeout=5)
            harness = RosLifecycleHarness(self.node, node_name)
            self.assertEqual(harness.state(), State.PRIMARY_STATE_UNCONFIGURED)
            self.assertFalse(harness.transition(Transition.TRANSITION_CONFIGURE))
            self.assertEqual(harness.state(), State.PRIMARY_STATE_UNCONFIGURED)

            harness.set_double_parameter("update_rate_hz", 20.0)
            self.assertTrue(harness.transition(Transition.TRANSITION_CONFIGURE))
            self.assertEqual(harness.state(), State.PRIMARY_STATE_INACTIVE)
            self.assertTrue(harness.transition(Transition.TRANSITION_ACTIVATE))
            self.assertEqual(harness.state(), State.PRIMARY_STATE_ACTIVE)
        finally:
            process.__exit__(None, None, None)
