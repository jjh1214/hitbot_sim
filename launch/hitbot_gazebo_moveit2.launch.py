from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import  ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution


def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "urdf", "Z-Arm_10042C0_gazebo.urdf.xacro"]
    )    

    world_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "worlds", "empty_world.world"]
    )

    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='urdf',
            default_value=urdf_path,
            description='URDF path'
        ),        

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "gazebo_robot_description", "-entity", "Z-Arm_10042C0",
                        "-x", "0.0", "-y", "0.0", "-z", "0.0"]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
            ],
            remappings=[('tf', 'gazebo_tf'), ('joint_states', 'gazebo_joint_states'), ('robot_description', 'gazebo_robot_description')]
        ),

        # Launch moveit2 after 5 seconds
        TimerAction(
            period=7.0,  # Delay for 5 seconds
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'hitbot_moveit2_config', 'demo.launch.py'],
                    output='screen'
                ),

            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/move_group', 'use_sim_time', 'true'],
                    output='screen'
                ),

            ]
        ),

    ])