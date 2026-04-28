#!/usr/bin/env -S ros2 launch
"""Example of planning with MoveIt2 in Gazebo using the chess tabletop world"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    black_model = LaunchConfiguration("black_model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    log_level = LaunchConfiguration("log_level")

    # List of included launch descriptions
    launch_descriptions = [
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ),
            launch_arguments=[("gz_args", [world, " -r -v ", gz_verbosity])],
        ),
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("panda_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("name", "panda"),
                ("namespace", ""),
                ("prefix", "panda_"),
                ("origin_xyz", "0 0 0"),
                ("origin_rpy", "0 0 0"),
                ("ros2_control_plugin", "gz"),
                ("ros2_control_command_interface", "effort"),
                # TODO: Re-enable collision geometry for manipulator arm once spawning with specific joint configuration is enabled
                ("collision_arm", "false"),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
        # Launch the namespaced MoveIt 2 stack for the black-side arm.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("panda_moveit_config"),
                        "launch",
                        "move_group_black.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("name", "black_panda"),
                ("namespace", "black"),
                ("prefix", "black_panda_"),
                ("origin_xyz", "1.2 0 0"),
                ("origin_rpy", "0 0 3.141592653589793"),
                ("ros2_control_plugin", "gz"),
                ("ros2_control_command_interface", "effort"),
                # TODO: Re-enable collision geometry for manipulator arm once spawning with specific joint configuration is enabled
                ("collision_arm", "false"),
                ("enable_rviz", "false"),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # ros_gz_sim_create (spawn the robot)
        Node(
            package="ros_gz_sim",
            executable="create",
            output="log",
            arguments=["-file", model, "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # Spawn the black-side arm on the opposite side of the board, rotated back toward it.
        Node(
            package="ros_gz_sim",
            executable="create",
            output="log",
            arguments=[
                "-file",
                black_model,
                "-name",
                "black_panda",
                "-x",
                "1.2",
                "-y",
                "0.0",
                "-z",
                "0.0",
                "-Y",
                "3.141592653589793",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    ld = LaunchDescription(declared_arguments + launch_descriptions + nodes)
    return ld


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World and model for Gazebo
        DeclareLaunchArgument(
            "world",
            default_value=path.join(
                get_package_share_directory("panda_description"),
                "worlds",
                "tabletop_chess.sdf",
            ),
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="panda",
            description="Name or filepath of model to load.",
        ),
        DeclareLaunchArgument(
            "black_model",
            default_value="black_panda",
            description="Name or filepath of the black-side robot model to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("panda_moveit_config"),
                "rviz",
                "moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "gz_verbosity",
            default_value="3",
            description="Verbosity level for Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]

