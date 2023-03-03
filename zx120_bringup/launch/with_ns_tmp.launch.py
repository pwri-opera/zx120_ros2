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

def generate_launch_with_ns(robot_name):
    # Get the launch directory
    deadtime_compensation_dir_path = get_package_share_directory("deadtime_compensation")
    excavator_pid_control_dir_path = get_package_share_directory("excavator_pid_control")
    
    # config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=robot_name, package_name=robot_name+"_moveit_config")
        .robot_description(file_path="config/"+robot_name+".urdf.xacro")
        .to_moveit_configs()
    )

    ### move group ###
    db_declare = DeclareBooleanLaunchArg(
        "db",
        default_value=False,
        description="By default, we do not start a database (it can be large)",
    )
    debug_declare = DeclareBooleanLaunchArg(
        "debug",
        default_value=False,
        description="By default, we are not in debug mode",
    )
    use_rviz_declare = DeclareBooleanLaunchArg("use_rviz", default_value=True)

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch_path = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch_path.exists():
        virtual_joints_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch_path)),
        )

    # Given the published joint states, publish tf for the robot links
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/rsp_remap.launch.py")
        ),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/move_group.launch.py")
        ),
    )

    # Run Rviz and load the default config to see the state of the move_group node
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/moveit_rviz_remap.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # If database loading was enabled, start mongodb as well
    warehouse_db_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/warehouse_db.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("db")),
    )

    # Fake joint driver
    fake_joint_driver = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
    )

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
        ),
    )
    ###
    
    ### deadtime compensation ###
    deadtime_compensation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(deadtime_compensation_dir_path, "launch/deadtime_compensation.launch.py")),
        launch_arguments={"dt_compensate": "True"}.items()
    )
    ###
    
    ### pid controller ###
    pid_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(excavator_pid_control_dir_path, "launch/zx120_pid.launch.py")),
        launch_arguments={"dt_compensate": "True",
                            "ff_enable": "False"}.items()
    )
    ###
    
    # ac58_joint_publisher
    ###
    
    # excavator_com3_ros
    ###
    
    ld = LaunchDescription()
    # ld.add_action(declare_robot_name)
    ld.add_action(db_declare)
    ld.add_action(debug_declare)
    ld.add_action(use_rviz_declare)
    
    # ld.add_action(virtual_joints_launch)
    ld.add_action(rsp_launch)
    ld.add_action(move_group_launch)
    ld.add_action(moveit_rviz_launch)
    # ld.add_action(warehouse_db_launch)
    # ld.add_action(fake_joint_driver)
    # ld.add_action(spawn_controllers_launch)
    
    # ld.add_action(deadtime_compensation_launch)
    
    # ld.add_action(pid_control_launch)
    
    return ld

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value=TextSubstitution(text="zx120")
    )

    # include another launch file in the chatter_ns namespace
    launch_include_with_namespace = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('robot_name')),
            generate_launch_with_ns("zx120"),
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        launch_include_with_namespace,
    ])