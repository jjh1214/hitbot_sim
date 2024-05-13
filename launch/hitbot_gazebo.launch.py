from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True

    world_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "worlds", "empty_world.world"]
    )

    # description_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('hitbot_sim'), 'launch', 'hitbot_rviz2.launch.py']
    # )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "Z-Arm_10042C0",
                        "-x", "0.0", "-y", "0.0", "-z", "0.0"]
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(description_launch_path),
        #     launch_arguments={
        #         'use_sim_time': str(use_sim_time),
        #         'publish_joints': 'true',
        #     }.items()
        # ),

    # # load_joint_state_broadcaster
	#     ExecuteProcess(
    #         cmd=['ros2', 'control', 'load_controller','joint_state_broadcaster'],
    #         output='screen'),

	# # load_joint_trajectory_controller
	#     ExecuteProcess( 
    #         cmd=['ros2', 'control', 'load_controller', 'joint_trajectory_controller'], 
    #         output='screen'),

    ])
