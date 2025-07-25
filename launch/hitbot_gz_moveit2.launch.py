import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time_param = {"use_sim_time": use_sim_time}

    def robot_state_publisher(context):
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('hitbot_sim'),
                    'urdf',
                    f'Z-Arm_10042C0_gazebo.urdf.xacro'
                ]),
            ]
        )
        robot_description = {'robot_description': robot_description_content}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, use_sim_time_param],
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('hitbot_moveit2_config'),
            'config',
            'hitbot_moveit_controllers.yaml',
        ],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'Z-Arm_10042C0', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--param-file',
                   robot_controllers,
                   ],
        parameters=[use_sim_time_param],
    )

    z_arm_moveit_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['z_arm_moveit_controller',
                   '--param-file',
                   robot_controllers,
                   ],
        parameters=[use_sim_time_param],
    )

    bridge_params = os.path.join(get_package_share_directory('hitbot_sim'),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
    )

    moveit_config = (
        MoveItConfigsBuilder("Z-Arm_10042C0", package_name="hitbot_moveit2_config")
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

    moveit_spawner = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time_param],
    )

    rviz_spawner = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            use_sim_time_param,
        ],
    )

    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[z_arm_moveit_controller_spawner],
            )
        ),  

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=z_arm_moveit_controller_spawner,
                on_exit=[moveit_spawner, rviz_spawner],
            )
        ),

        ros_gz_bridge,
        gz_spawn_entity,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

    ])
    
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld