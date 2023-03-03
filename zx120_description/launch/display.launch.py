import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory(
            "zx120_description"), "launch", "display.rviz")

    xacro_model = os.path.join(
        get_package_share_directory(
            "zx120_description"), "urdf", "zx120.xacro")

    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            namespace='zx120',
            executable='robot_state_publisher',
            output="screen",
            parameters=[params],
            # remappings=[('/tf', 'tf'),
            #         ('/tf_static', 'tf_static'),],
        ),
        Node(
            package='joint_state_publisher_gui',
            namespace='zx120',
            executable='joint_state_publisher_gui',
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace='zx120',
            arguments=["--display-config", rviz_config],
            output="screen",
            remappings=[
                # ('/tf', 'tf'),
                # ('/tf_static', 'tf_static'),
                ('/robot_description', 'robot_description'),
                # ('/initialpose', 'initialpose'),
                # ('/clicked_point', 'clicked_point'),
                # ('/goal_pose', 'goal_pose'),
            ],
        ),
    ])
