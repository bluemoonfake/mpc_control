from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("mpc_controller")
    controller_config = PathJoinSubstitution([
        package_share, "config", "controller.yaml"
    ])
    default_mission = PathJoinSubstitution([
        package_share, "config", "missions", "benchmark_square.json"
    ])
    mission_file_path = LaunchConfiguration("mission_file_path")
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="mpc",
                              choices=["mpc", "px4_pid"]),
        DeclareLaunchArgument(
            "mission_file_path",
            default_value=default_mission,
            description="Absolute or relative path to the mission JSON file",
        ),
        Node(
            package="mpc_controller",
            executable="reference_generator_node",
            name="reference_generator_node",
            parameters=[controller_config, {"mission_file_path": mission_file_path}],
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
            condition=LaunchConfigurationEquals("controller", "mpc"),
            name="mpc_controller_node",
            parameters=[controller_config],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="px4_attitude_mode_node",
            condition=LaunchConfigurationEquals("controller", "mpc"),
            name="px4_attitude_mode_node",
            parameters=[controller_config],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="pid_mode_node",
            name="pid_mode_node",
            condition=LaunchConfigurationEquals("controller", "px4_pid"),
            parameters=[controller_config],
            output="screen",
        ),
    ])
