from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "urdf", "Z-Arm_10042C0_gazebo.urdf.xacro"]
    )    

    world_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "worlds", "empty_world.world"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('hitbot_sim'), 'rviz', 'default.rviz']
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

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Run rviz'
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
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "Z-Arm_10042C0",
                        "-x", "0.0", "-y", "0.0", "-z", "0.0"]
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller','joint_state_broadcaster', '--set-state', 'active'],
            output='screen'
        ),

        ExecuteProcess( 
            cmd=['ros2', 'control', 'load_controller', 'z_arm_controller', '--set-state', 'active'], 
            output='screen'
        ),            

    ])