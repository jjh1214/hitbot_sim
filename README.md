# Hitbot_sim
[![license - MIT](https://img.shields.io/:license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple.svg)](https://index.ros.org/doc/ros2/Releases/)

# Note
This repository is ROS2-Jazzy Package for the [Z-Arm of Hitbot.](https://www.hitbotrobot.com/category/product-center/4-axis-robot-arm/)

This repository is made with an upgrade from gazebo classic to [gazebo harmonic.](https://gazebosim.org/docs/harmonic/getstarted/)



# Build
### *This Package is implemented at ROS2-Jazzy.*
### *Please install Gazebo Harmonic before Build.*
```
### I assume that you have installed the ros-jazzy-desktop package using the apt-get command.
### I assume that you have installed Gazebo Harmonic
### I recommand the /home/<user_home>/hitbot_ws/src
### Before activate simulation, please install moveit2 and gazebo harmonic



$ mkdir -p ~/hitbot_ws/src
$ cd ~/hitbot_ws/src
$ git clone https://github.com/jjh1214/hitbot.git
$ git clone -b jazzy-devel https://github.com/jjh1214/hitbot_sim.git
$ git clone https://github.com/jjh1214/hitbot_msgs.git
$ git clone -b jazzy https://github.com/jjh1214/hitbot_moveit2_config.git

$ sudo apt-get install ros-jazzy-joint-state-publisher-gui

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
[Screencast from 2025-07-25 11-00-50.webm](https://github.com/user-attachments/assets/1c028cb6-4dc7-4bd2-8166-9d44fac37a5d)


### If you want to move model
```
ros2 topic pub -1 /z_arm_position_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data:
  - -0.5
  - 0.0
  - 0.0
  - 0.0"
```

# Run - GAZEBO harmonic with rviz2
```
$ ros2 launch hitbot_sim hitbot_gz_rviz.launch.py
```
[Screencast from 2025-07-25 11-02-34.webm](https://github.com/user-attachments/assets/ee88d6df-d7d3-478f-a8d5-0872eeb6c543)

### If you want to move model
```
ros2 topic pub -1 /z_arm_position_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data:
  - -0.5
  - 0.0
  - 0.0
  - 0.0"
```

# Run - GAZEBO harmonic with Moveit2
```
$ ros2 launch hitbot_sim hitbot_gz_moveit2.launch.py
```
[Screencast from 2025-07-25 11-04-11.webm](https://github.com/user-attachments/assets/a12a132a-d438-4c7e-9197-db726434b89c)

