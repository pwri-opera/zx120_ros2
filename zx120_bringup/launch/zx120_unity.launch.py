import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import yaml
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("zx120", package_name="zx120_moveit_config")
        .robot_description(file_path="config/zx120.urdf.xacro")
        .robot_description_semantic(file_path="config/zx120.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("zx120_description"),
    #                 "urdf",
    #                 "zx120.xacro",
    #             ]
    #         ),
    #     ]
    # )
    # robot_description = {"robot_description": robot_description_content}
    # robot_description = {"robot_description": moveit_config.robot_description}

    run_move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/move_group.launch.py")
        ),
    )

    run_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/rsp.launch.py")
        ),
    )


    run_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
        ),
        # condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'])


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("zx120_control_hardware"),
            "config",
            "zx120_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, robot_controllers],
        # parameters=[robot_description, robot_controllers],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["upper_arm_controller"],
    )

    return LaunchDescription([
        run_move_group,
        run_rviz,
        static_tf,
        run_robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
