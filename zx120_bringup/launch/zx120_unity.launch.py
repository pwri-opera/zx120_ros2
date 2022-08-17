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

from launch_ros.actions import Node, SetParameter


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # moveit_config = (
    #     MoveItConfigsBuilder("zx120", package_name="zx120_moveit_config")
    #     .robot_description(file_path="config/zx120_unity.urdf.xacro")
    #     .robot_description_semantic(file_path="config/zx120.srdf")
    #     .robot_description_kinematics(file_path="config/kinematics.yaml")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .planning_scene_monitor(
    #         publish_robot_description=True, publish_robot_description_semantic=True
    #     )
    #     .planning_pipelines(pipelines=["ompl"])
    #     .to_moveit_configs()
    # )

    # set_use_sim_time = SetParameter(
    #     name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    use_sim_time=False

    # run_move_group = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         str(moveit_config.package_path / "launch/move_group.launch.py"),
    #     ),
    #     # launch_arguments={'use_sim_time': use_sim_time}.items(),
    # )

    # run_robot_state_publisher = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         str(moveit_config.package_path / "launch/rsp.launch.py")
    #     ),
    # )


    # run_rviz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
    #     ),
    #     # condition=IfCondition(LaunchConfiguration("use_rviz")),
    # )

    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
                     parameters=[{'use_sim_time': use_sim_time}])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("zx120_moveit_config"),
                    "config",
                    "zx120_unity.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # sim_description = {"use_sim_time": true}

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
        parameters=[robot_description, robot_controllers,
                    {'use_sim_time': use_sim_time}],
        # parameters=[moveit_config.robot_description, robot_controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # name="jsb_spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # name="uac_spawner",
        arguments=["upper_arm_controller",
                   "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # move_group 
    robot_description_semantic_config = load_file(
        'zx120_moveit_config', 'config/zx120.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml(
        'zx120_moveit_config', 'config/kinematics.yaml')

    ompl_planning_pipeline_config = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization \
                               default_planner_request_adapters/FixWorkspaceBounds \
                               default_planner_request_adapters/FixStartStateBounds \
                               default_planner_request_adapters/FixStartStateCollision \
                               default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1}}
    ompl_planning_yaml = load_yaml(
        'zx120_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_controllers = load_yaml(
        'zx120_moveit_config', 'config/moveit_controllers.yaml')

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.1}

    planning_scene_monitor_parameters = {'publish_planning_scene': True,
                                         'publish_geometry_updates': True,
                                         'publish_state_updates': True,
                                         'publish_transforms_updates': True}

    run_move_group_node = Node(package='moveit_ros_move_group',
                               executable='move_group',
                               output='screen',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           trajectory_execution,
                                           moveit_controllers,
                                           planning_scene_monitor_parameters,
                                           {'use_sim_time': use_sim_time},
                                           ])

    # rviz
    rviz_config_file = get_package_share_directory(
        'zx120_moveit_config') + '/launch/moveit.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 ompl_planning_pipeline_config,
                                 kinematics_yaml,
                                 {'use_sim_time': use_sim_time},
                                 ])

    
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description, {'use_sim_time': use_sim_time}])

    return LaunchDescription([
        # run_move_group,
        # run_rviz,
        static_tf,
        # run_robot_state_publisher,
        run_move_group_node,
        rviz_node,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
