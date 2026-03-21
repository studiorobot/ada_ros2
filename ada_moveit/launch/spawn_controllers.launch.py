# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    # Log Level
    log_level_da = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")
    log_level_cmd_line_args = ["--ros-args", "--log-level", log_level]
    ld.add_action(log_level_da)

    # Spawn the joint_state_broadcaster first (not part of MoveIt controllers)
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"] + log_level_cmd_line_args,
        )
    )

    # Spawn ONLY the default controller (joint_trajectory_controller) as ACTIVE
    # Other controllers (jaco_arm_controller, jaco_arm_servo_controller, jaco_arm_cartesian_controller)
    # are defined in mock_kortex_controllers.yaml and can be loaded dynamically via switch_controller
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller"] + log_level_cmd_line_args,
        )
    )

    # Pre-load the force gate controller in inactive state
    # This ensures the /jaco_arm_controller/set_parameters service exists
    # The behavior tree can then activate it dynamically via switch_controller
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["jaco_arm_controller", "--inactive"] + log_level_cmd_line_args,
        )
    )

    # Pre-load the force gate position controller (servo) in inactive state
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["jaco_arm_servo_controller", "--inactive"] + log_level_cmd_line_args,
        )
    )

    # Pre-load the cartesian twist controller in inactive state
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["jaco_arm_cartesian_controller", "--inactive"] + log_level_cmd_line_args,
        )
    )

    return ld
