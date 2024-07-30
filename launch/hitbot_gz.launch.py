from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "urdf", "Z-Arm_10042C0_gazebo.urdf.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('hitbot_sim'), 'rviz', 'default.rviz']
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
            name='publish_joints',
            default_value='true',
            description='Launch joint_states_publisher'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints")),
            parameters=[
                # {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    # 'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
            ],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name='joint_state_publisher_gui',
            output="screen"
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': False}]
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            parameters=[robot_controllers],
            output="both",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
        ),

        Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # namespace=PathJoinSubstitution([LaunchConfiguration('name'), "gz"]),
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "Z_arm",
            "-allow_renaming",
            "true",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "0",
        ],
    ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    name="controller_manager",
                    parameters=[robot_controllers],
                    output="both",
                ),
                on_exit=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                        output="screen"
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["z_arm_moveit_controller", "-c", "/controller_manager"],
                        output="screen"
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["z_arm_joint_publisher", "-c", "/controller_manager"],
                        output="screen"
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["z_arm_position_controller", "-c", "/controller_manager"],
                        output="screen"
                    )
                ]
            )
        )
    ])