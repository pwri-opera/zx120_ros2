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

def generate_demo_launch_remap_rviz(moveit_config):
    """
    Launches a self contained demo

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    # virtual_joints_launch = (
    #     moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    # )
    # if virtual_joints_launch.exists():
    #     ld.add_action(
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(str(virtual_joints_launch)),
    #         )
    #     )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld

def generate_launch_description():
    
    robot_name="zx120"    
    # Define arguments for launch files
    # robot_name = LaunchConfiguration("robot_name")
    # declare_robot_name = DeclareLaunchArgument(
    #     "robot_name",
    #     default_value="zx120",
    #     description="Name of a robot"
    # )
    
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

    # move group
    moveit_demo_launch = generate_demo_launch(moveit_config)
    ###
    
    # deadtime compensation
    deadtime_compensation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(deadtime_compensation_dir_path, "launch/deadtime_compensation.launch.py")),
        launch_arguments={"dt_compensate": "True"}.items()
    )
    ###
    
    # pid controller
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
    
    # # ros2 control
    # zx120_controllers = os.path.join(
    #     get_package_share_directory('zx120_control_hardware'),
    #     'config',
    #     'zx120_controllers.yaml'
    # )
        
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #       moveit_config.robot_description,
    #       zx120_controllers
    #       ],
    #     output='screen',
    # )

    # spawn_joint_state_broadcaster = ExecuteProcess(
    #             cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
    #             shell=True,
    #             output='screen',
    # )

    # spawn_upper_arm_controller = ExecuteProcess(
    #             cmd=['ros2 run controller_manager spawner upper_arm_controller'],
    #             shell=True,
    #             output='screen',
    # )
    # ###
    
    ld = LaunchDescription()
    # ld.add_action(declare_robot_name)
    
    ld.add_action(moveit_demo_launch)
    ld.add_action(deadtime_compensation_launch)
    ld.add_action(pid_control_launch)
    # ld.add_action(controller_manager)
    # ld.add_action(spawn_joint_state_broadcaster)
    # ld.add_action(spawn_upper_arm_controller)

    return ld
