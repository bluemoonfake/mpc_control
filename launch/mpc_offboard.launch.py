from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("mpc_controller")
    controller_config = PathJoinSubstitution([
        package_share, "config", "controller.yaml"
    ])
    return LaunchDescription([
        Node(
            package="mpc_controller",
            executable="reference_generator_node",
            name="reference_generator_node",
            parameters=[controller_config],
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
            parameters=[controller_config],
            output="screen",
        ),
        Node(
            package="mpc_controller",
            executable="px4_attitude_mode_node",
            name="px4_attitude_mode_node",
            parameters=[controller_config],
            output="screen",
        ),
    ])
