import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import  ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_moveit2_config"), "config", "Z-Arm_10042C0.urdf.xacro"]
    )    

    world_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "worlds", "empty_world.world"]
    )

    moveit_config = (
        MoveItConfigsBuilder("Z-Arm_10042C0", package_name="hitbot_moveit2_config")
        .robot_description(file_path="config/Z-Arm_10042C0.urdf.xacro")
        .robot_description_semantic(file_path="config/Z-Arm_10042C0.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('hitbot_moveit2_config'), 'config', 'moveit.rviz']
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hitbot_sim"),
            "config",
            "hitbot_controller2.yaml",
        ]
    )

    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='urdf',
            default_value=urdf_path,
            description='URDF path'
        ),        

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    # 'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                    
                }
            ],
            remappings=[('tf', 'gazebo_tf'), ('joint_states', 'gazebo_joint_states'), ('robot_description', 'gazebo_robot_description')]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                moveit_config.robot_description,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),

        Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
            ),

        Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_path],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,],
            ),

        Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                parameters=[
                    moveit_config.robot_description,  
                    moveit_config.trajectory_execution,
                ],
                output="both",
            ),

        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         ExecuteProcess(
        #             cmd=['ros2', 'control', 'load_controller', 'joint_state_broadcaster', '--set-state', 'active'],
        #             output='screen'
        #         ),
        #         ExecuteProcess(
        #             cmd=['ros2', 'control', 'load_controller', 'z_arm_moveit_controller', '--set-state', 'active'],
        #             output='screen'
        #         ),
        #     ]
        # )

        # Node(
        #         package="controller_manager",
        #         # namespace=LaunchConfiguration('name'),
        #         executable="spawner",
        #         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        #     ),

    ])