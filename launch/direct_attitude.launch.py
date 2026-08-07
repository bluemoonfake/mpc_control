from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="mpc_controller", executable="reference_generator_node", output="screen"),
        Node(package="mpc_controller", executable="vehicle_state_bridge_node", output="screen"),
        Node(package="mpc_controller", executable="mpc_controller_node", output="screen"),
        Node(package="mpc_controller", executable="px4_attitude_setpoint_node", output="screen"),
    ])
