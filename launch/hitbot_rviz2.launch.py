import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = get_package_share_directory("hitbot_sim")
    urdf_file = os.path.join(pkg_path, "urdf", "Z-Arm_10042C0.urdf")
    with open(urdf_file, "r") as file:
        robot_description = file.read()

    params = {"robot_description": robot_description, "use_sim_time": use_sim_time}

    rviz_config_file = get_package_share_directory('hitbot_sim') + "/rviz/default.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="use sim time"
            ),

            # rviz2
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config_file]
                ),

            # robot_state_publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params]
                ),

            # Static TF
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                output='log',
                arguments=['0.0', '0.0', '0.0', '3.1416', '3.1416', '1.5708', 'world', 'base_link']
                ),

            # joint_state_publisher_gui
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen"
                ),

        ]
    )