import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    robot_name = "zx120"
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=robot_name, package_name=robot_name+"_moveit_config")
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
