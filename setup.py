import os
from glob import glob
from setuptools import setup

package_name = 'hitbot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'sensors'), glob('urdf/sensors/*.xacro')),
        (os.path.join('share', package_name, 'meshes', 'Z-Arm_10042C0'), glob('meshes/Z-Arm_10042C0/*.dae')),
        (os.path.join('share', package_name, 'models', 'Z-Arm_10042C0'), glob('models/Z-Arm_10042C0/*.dae')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.model')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev_jung',
    maintainer_email='seedn7777@cwsfa.co.kr',
    description='Hitbot Z-Arm robot simulation with ROS2 and Rviz2, Gazebo, Moveit2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'hitbot_controller_joint_state       = hitbot_sim.hitbot_controller_joint_state:main',
            'hitbot_controller_gazebo_pos        = hitbot_sim.hitbot_controller_gazebo_pos:main',
            
        ],
    },
)
