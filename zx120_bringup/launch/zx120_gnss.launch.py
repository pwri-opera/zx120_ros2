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
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

def generate_launch_description():
    nmea_serial_driver_node=Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        name="nema_serial_driver_node",
        namespace="gnss_compass",
        parameters=[
            {"port": "/dev/ttyS0"},
            {"baud": 115200},
            {"frame_id": "map" },
            {"use_GNSS_time": False},
            {"time_ref_source": "gps"},
            {"useRMC": False},
        ],
        output="screen",
    )
    fix2tfpose_node=Node(
        package="gnss_poser",
        executable="gnss_poser",
        name="fix2tfpose",
        namespace="gnss_compass",
        parameters=[{"plane_zone": 9},
                    {"coordinate_system": 2}], # plane_zoneを反映
        remappings=[("/gnss_pose", "gnss_compass/gnss_pose"),
        ],
        output="screen",
    )
    
    ld = LaunchDescription()
    ld.add_action(nmea_serial_driver_node)
    ld.add_action(fix2tfpose_node)
    return ld