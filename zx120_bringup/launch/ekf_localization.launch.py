import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition


def generate_launch_description():

    ekf_yaml = os.path.join(
        get_package_share_directory('zx120_bringup'),
        'config',
        'zx120_ekf.yaml'
    )

    pose_stamped2_odometry_node = Node(
        package="zx120_bringup",
        name="poseStamped2Odometry",
        executable="poseStamped2Odometry.py",
    )
    ekf_filter_node = Node(
        package="robot_localization",
        name="ekf_filter_node",
        executable="ekf_node",
        output="screen",
        parameters=[
            ekf_yaml,
            {"tf_prefix": ""},
            {"odom_frame": "odom"},
            {"base_link_frame": "base_link"},
        ],
        remappings=[("odom0","odom2baselink"),
                    ("odom1", "gnss_odom"),
        ],
    )
    
    ld = LaunchDescription()
    ld.add_action(pose_stamped2_odometry_node)
    ld.add_action(ekf_filter_node)
    
    return ld
