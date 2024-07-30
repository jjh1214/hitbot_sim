from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("hitbot_sim"), "urdf", "Z-Arm_10042C0_gazebo.urdf.xacro"]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hitbot_sim"),
            "config",
            "hitbot_control_gz.yaml",
        ]
    )    

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[robot_controllers],
        output="both",
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
        ),

        controller_manager_node,

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

        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'joint_state_broadcaster', '--set-state', 'active'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'z_arm_position_controller', '--set-state', 'active'],
                    output='screen'
                ),
            ]
        )
    ])