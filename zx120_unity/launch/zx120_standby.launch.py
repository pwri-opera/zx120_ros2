import pathlib

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    ld = LaunchDescription()
    # # Get the launch directory
    # zx120_unity_share_dir = get_package_share_directory("zx120_unity")

    # Create the launch configuration variables
    # robot_name = LaunchConfiguration("robot_name")
    # init_pose = LaunchConfiguration("init_pose")
    # use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare the launch configuration variables
    # declare_robot_name_cmd = DeclareLaunchArgument(
    #     "robot_name",
    #     default_value="zx120",
    #     description="Name of the robot"
    # )
    # declare_init_pose_cmd = DeclareLaunchArgument(
    #     "init_pose",
    #     default_value="-x 0 -y 0 -z 0",
    #     description="Initial position of the robot"
    # )
    # declare_use_sim_time_cmd = DeclareLaunchArgument(
    #     "use_sim_time",
    #     default_value="true",
    #     description="Whether to use sim_time or wall_time"
    # )

    # Create moveit demo launch
    #  Includes
    #  * static_virtual_joint_tfs
    #  * robot_state_publisher
    #  * move_group
    #  * moveit_rviz
    #  * warehouse_db (optional)
    #  * ros2_control_node + controller spawners
    robot_name = "zx120"
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=robot_name, package_name=robot_name+"_moveit_config")
        .robot_description(file_path="config/"+robot_name+"_unity.urdf.xacro")
        .to_moveit_configs()
    )
    # TODO: Set init_pose, use_sim_time
    moveit_demo_launch = generate_demo_launch(moveit_config)

    # ld.add_action(declare_robot_name_cmd)
    ld.add_action(moveit_demo_launch)

    return ld
