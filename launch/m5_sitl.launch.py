from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("mpc_controller"), "config", "m5_hold.yaml"
    ])

    hold_current_state = LaunchConfiguration("hold_current_state_on_enable")

    return LaunchDescription([
        DeclareLaunchArgument(
            "hold_current_state_on_enable",
            default_value="true",
            description="Capture the first control-ready VehicleState as the hold reference"),
        Node(
            package="mpc_controller",
            executable="reference_generator_node",
            name="reference_generator_node",
            parameters=[
                config,
                {"hold_current_state_on_enable": ParameterValue(
                    hold_current_state, value_type=bool)}],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="vehicle_state_bridge_node",
            name="vehicle_state_bridge_node",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="mpc_controller_node",
            name="mpc_controller_node",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="px4_wrench_bridge_node",
            name="px4_wrench_bridge_node",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="m5_safety_monitor.py",
            name="m5_safety_monitor",
            output="screen",
        ),
    ])
