from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("mpc_controller")
    controller_config = PathJoinSubstitution([
        package_share, "config", "controller.yaml"
    ])
    offboard_config = PathJoinSubstitution([
        package_share, "config", "offboard.yaml"
    ])
    hold_current_state = LaunchConfiguration("hold_current_state_on_enable")

    return LaunchDescription([
        DeclareLaunchArgument(
            "hold_current_state_on_enable",
            default_value="true",
            description="Capture and hold the current position before Offboard"),
        Node(
            package="mpc_controller",
            executable="reference_generator_node",
            name="reference_generator_node",
            parameters=[
                controller_config,
                offboard_config,
                {"hold_current_state_on_enable": ParameterValue(
                    hold_current_state, value_type=bool)}],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="vehicle_state_bridge_node",
            name="vehicle_state_bridge_node",
            parameters=[controller_config],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="mpc_controller_node",
            name="mpc_controller_node",
            parameters=[controller_config, offboard_config],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="px4_attitude_setpoint_node",
            name="px4_attitude_setpoint_node",
            parameters=[offboard_config],
            output="screen",
        ),
    ])
