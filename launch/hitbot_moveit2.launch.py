from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import  ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "urdf", "Z-Arm_10042C0_gazebo.urdf.xacro"]
    )    

    world_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "worlds", "empty_world.world"]
    )

    moveit_config = (
        MoveItConfigsBuilder("m1013")
        .robot_description(file_path="config/m1013.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
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
            # remappings=[('tf', 'gazebo_tf'), ('joint_states', 'gazebo_joint_states'), ('robot_description', 'gazebo_robot_description')]
        ),

        Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict()],
            ),

    ])