# Hitbot_sim
[![license - MIT](https://img.shields.io/:license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-red.svg)](https://index.ros.org/doc/ros2/Releases/)

# Note
This repository is ROS2-Humble Package for the [Z-Arm of Hitbot.](https://www.hitbotrobot.com/category/product-center/4-axis-robot-arm/)

This repository is made with an upgrade from gazebo classic to [gazebo harmonic.](https://gazebosim.org/docs/harmonic/getstarted/)



# Build
### *This Package is implemented at ROS2-Humble.*
### *Please install Gazebo Harmonic before Build.*
```
### I assume that you have installed the ros-humble-desktop package using the apt-get command.
### I assume that you have installed Gazebo Harmonic
### I recommand the /home/<user_home>/hitbot_ws/src
### Before activate simulation, please install moveit2 and gazebo



$ mkdir -p ~/hitbot_ws/src
$ cd ~/hitbot_ws/src
$ git clone -b humble https://github.com/jjh1214/hitbot.git
$ git clone -b jazzy-devel https://github.com/jjh1214/hitbot_sim.git
$ git clone -b humble https://github.com/jjh1214/hitbot_msgs.git
$ git clone -b humble https://github.com/jjh1214/hitbot_moveit2_config.git

$ sudo apt-get install ros-humble-joint-state-publisher-gui

$ cd ~/hitbot_ws
$ colcon build
$ . install/setup.bash
$ source hitbot_ws/install/local_setup.bash



### if you want
# $ echo 'source ~/hitbot_ws/install/local_setup.bash' >> ~/.bashrc 
```

# Run - GAZEBO harmonic
```
$ ros2 launch hitbot_sim hitbot_gz.launch.py
```
[Screencast from 08-12-2024 10:21:46 AM.webm](https://github.com/user-attachments/assets/d85c97d9-7229-4986-853c-3fa68db5e035)

### If you want to move model
```
   ros2 topic pub -1 /z_arm_moveit_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
     stamp:
       sec: 0
       nanosec: 0
     frame_id: ''
   joint_names: ['joint1', 'joint2', 'joint3', 'joint4']
   points: [{positions: [-0.1, 0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0, 0.0], accelerations: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 1, nanosec: 0}}]"
```

# Run - GAZEBO harmonic with rviz2
```
$ ros2 launch hitbot_sim hitbot_gz_rviz.launch.py
```
[Screencast from 08-12-2024 10:07:55 AM.webm](https://github.com/user-attachments/assets/913d8ef8-bfe4-4cb6-ba99-fd849091b036)
### If you want to move model
```
   ros2 topic pub -1 /z_arm_moveit_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
     stamp:
       sec: 0
       nanosec: 0
     frame_id: ''
   joint_names: ['joint1', 'joint2', 'joint3', 'joint4']
   points: [{positions: [-0.1, 0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0, 0.0], accelerations: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 1, nanosec: 0}}]"
```

# TODO
### Fix hitbot_moveit_config pkg. (Humble -> Jazzy)
### Chage move commands. (joint_trajectory -> position_controller)
### Clear Code. (Delete humble code)
### Update ReadME.