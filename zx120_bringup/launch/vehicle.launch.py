import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.substitutions import PathJoinSubstitution, TextSubstitution



def generate_launch_description():
    # config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="zx120", package_name="zx120_moveit_config")
        # .robot_description(file_path="config/"+"zx120_unity.urdf.xacro")
        .robot_description(file_path="config/"+"zx120.urdf.xacro")
        .to_moveit_configs()
    )

    # move group
    moveit_demo_launch = generate_demo_launch(moveit_config)
    ###
    
    # ros2 control
    zx120_controllers = os.path.join(
        get_package_share_directory('zx120_control_hardware'),
        'config',
        'zx120_controllers.yaml'
    )
        
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
          moveit_config.robot_description,
          zx120_controllers
          ],
        output='screen',
    )

    spawn_joint_state_broadcaster = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
                shell=True,
                output='screen',
    )

    spawn_upper_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner upper_arm_controller'],
                shell=True,
                output='screen',
    )
    ###
    
    ld = LaunchDescription()
    # ld.add_action(moveit_demo_launch)
    ld.add_action(controller_manager)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(spawn_upper_arm_controller)

    return ld
