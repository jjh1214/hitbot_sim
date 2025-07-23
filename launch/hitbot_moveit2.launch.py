import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_moveit2_config"), "config", "Z-Arm_10042C0.urdf.xacro"]
    )

    moveit_config = (
        MoveItConfigsBuilder("Z-Arm_10042C0", package_name="hitbot_moveit2_config")
        .robot_description(file_path="config/Z-Arm_10042C0.urdf.xacro")
        .robot_description_semantic(file_path="config/Z-Arm_10042C0.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('hitbot_moveit2_config'), 'config', 'moveit.rviz']
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hitbot_moveit2_config"), "config", "hitbot_moveit_controllers.yaml"]
    )

    ld.add_action(DeclareLaunchArgument(
        name='urdf',
        default_value=urdf_path,
        description='URDF path'
    ))

    ld.add_action(DeclareLaunchArgument(
        name='db',
        default_value='False',
        description='If true, run warehouse MongoDB server'
    ))

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
        ],
    ))

    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            moveit_config.robot_description,
        ],
    ))

    ld.add_action(Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    ))

    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    ))

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        # remappings=[
        #     ("/controller_manager/robot_description", "/robot_description"),
        # ],
        output="screen",
    )
    ld.add_action(controller_manager_node)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager",
                   "/controller_manager"],
        output="screen"
    )

    z_arm_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["z_arm_moveit_controller",
                   "-c",
                   "/controller_manager"],
        output="screen"
    )

    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner, z_arm_moveit_controller_spawner],
        )
    ))

    db_config = LaunchConfiguration("db")
    ld.add_action(Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    ))

    return ld