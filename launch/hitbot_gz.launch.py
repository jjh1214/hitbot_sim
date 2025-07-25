import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():

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
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [robot_state_publisher_node]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('hitbot_sim'),
            'config',
            'hitbot_controller2.yaml',
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
    )

    z_arm_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['z_arm_position_controller',
                   '--param-file',
                   robot_controllers,
                   ],
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
                on_exit=[z_arm_position_controller_spawner],
            )
        ),        

        ros_gz_bridge,
        gz_spawn_entity,

    ])

    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld