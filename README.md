# my_robot

# ROS2 Robot Simulation and Control
This repository contains a ROS2 workspace with packages to simulate a differential drive robot with a Lidar sensor and RGB camera, and control the robot using keyboard commands.
## Prerequisites
- ROS2 Foxy (or a newer version) installed on your system.
- gazebo_ros
- robot_state_publisher
- rviz2
- teleop_twist_keyboard
- sensor_msgs
- geometry_msgs
- std_msgs
- rclcpp

You can install these packages using the following command:
```bash
sudo apt install ros-<your_distro>-<package_name>
```
## Workspace Setup
### 1. Create your workspace and src folder:
```bash
mkdir -p [your_name]_ws/src
cd [your_name]_ws/src
```
### 2. Clone this repository into the src folder:
```bash
git clone https://github.com/Cheguevara-Sureshkumar/my_robot.git
```
### 3. Build workspace:
```bash
cd ../
colcon build --symlink-install
```
## Running Simulation:
### 1. Open a new terminal and source the workspace setup files:
```bash
source install/setup.bash
```
### 2. Launch the Gazebo world and spawn the robot:
```bash
ros2 launch bot_world bot_test_world.launch.py
```
### 3. In another terminal, launch the RViz visualization:
```bash
ros2 launch bot_spawn rviz.launch.py
```
### 4. In a third terminal, launch the teleoperation control:
```bash
ros2 launch bot_spawn control.launch.py
```
### 5. In a fourth terminal, launch the Lidar data reading and filtering:
```bash
ros2 launch bot_control laser_scan.launch.py
```

This will start all the necessary components, including the Gazebo simulation, RViz visualization, robot teleoperation, and Lidar data processing. You can use the keyboard commands to control the robot, observe the robot's movement and Lidar data in RViz, and inspect the filtered Lidar data on the /filtered_scan topic.
