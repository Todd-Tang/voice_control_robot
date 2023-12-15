# Voice Control Autonomous Robot with Sound Source Localization
#### Contributor: Todd Tang
#### Last time updated: Dec. 15, 2023

## I. Preparation
### 1. Make sure you have ROS2-humble properly installed.
Follow instructions on ros.org:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### 2. Install dependencies
```bash
# install gazebo
sudo apt install ros-humble-gazebo-*
# install cartographer
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
# install navigation2
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
# install turtlebo3
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
# install pyroomacoustics
pip install pyroomacoustics
```

### 3. Modify .bashrc to have those lines at the end of the file:
```bash
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp #optional
source /opt/ros/humble/setup.bash
```
### 4. Create a workspace; Install packages
```bash
mkdir ~/humble_ws/src
cd ~/humble_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/Todd-Tang/voice_control_robot.git
```
### 5. You **must** modify a yaml file
Locate `src/path_planner_server/config/bt_navigator.yaml`

Find `default_nav_to_pose_bt_xml`
```yaml
default_nav_to_pose_bt_xml: "/home/toddtang/wr_ws/src/path_planner_server/config/behavior.xml"
```

Change its value to the directory path that matches with your file system

### 6. Compile your workspace

**Always Compile in the Workspace!**
```bash
cd ~/humble_ws
colcon build
source install/setup.bash
```
## II. Launch

### Step 1. Launch Gazebo Simulation
```bash
# Terminal 1
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
```
### Step 2. Launch navigation and sound source localization
```bash
# Terminal 2
source install/setup.bash
ros2 launch wr_package wr_package_launch_file.launch.py 
```

### Step 3. Select the sound location in Rviz
In Rviz Interface:

Click 'Publish Point"

Click on the map where you want to place the sound source

### Step 4. Control the robot
#### Option 1: Control by text commands:
```bash
# Terminal 3
source install/setup.bash

ros2 topic pub --once /voice_command custom_interfaces/msg/VoiceCommand "{command: come, number: 3, unit: meters}"
# Command type: come, down, up, left, right, stop
```
#### Option 2: Control by voice commands:
1. Wait until the simulations are opened:
```bash
# Terminal 3
source install/setup.bash

ros2 run wr_package voice_command_node 
```
2. When you see "Listening..." on Terminal 3:

3. Say: Hey Google to start voice controlling. 

4. After you hear back, say commands like `Can you come close to me by 2 meters?` Then check if console outputs the correct commands to execute, e.g., [come, 2, meters]. 

5. Then the robot should be moving towards the location of the sound source (that you set in Rviz) by 2 meters.




## Interesting Findouts

### 1. Rviz2 adding display issue

For unknown reasons, when adding new display to Rviz2 interface (such as Map) from display type, sometimes it will give Status Error saying: error subscribing: empty topic name. It won't let you type the topic name or choose from dropdown button either. In such cases, instead of adding display from display type, you add from topic name and then you are able to type or choose your topic name.

#### General solution to above issue:
1. Save the current rviz config
2. Open the config file (.rviz), locate those displays that show the error in the code
   For example, if ParticleCloud is not working, locate the code: 
   Class: nav2_rviz_plugins/ParticleCloud
3. Scroll down until find "Topic", inside which the value would be "", which is empty.
4. Replace the "" with correct topic name, which can be found on ros.org
   For example, for ParticleCloud, the code would be updated as:
   Value: /particlecloud
5. Restart Rviz2, and open the updated config file


### 2. Ubuntu 22.04 Graphics Problem

#### General solution to above issue:
https://askubuntu.com/questions/1403595/ubuntu-22-04-graphics-problem

