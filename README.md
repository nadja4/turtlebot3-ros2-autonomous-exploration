# ROS 2 Turtlebot 3 Map Explorer (Real Robot)
## Description
This repository is a fork of Daniel Garcia Lopez's [repository](https://github.com/DaniGarciaLopez/ros2_explorer), which implements exploration for a simulated turtlebot3.

## Installation (tested on Ubuntu 22.04 - ROS 2 Humble)

Install Turtlebot3 and ROS2 Humble Packages as descripted in the Quick-Start-Guide (link below). 

When installing TurtleBot3 Packages make sure you build them from source (*not* with sudo apt ...)

[Quick-Start-Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

Don't forget to install colcon:
```
sudo apt install python3-colcon-common-extensions
```
Install Python libraries:
```
sudo apt install python3-pip
pip3 install pandas
```
Create a ROS2 workspace:
```
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
```
Clone the repository:
```
git clone <link-to-repo>
```
Compile packages and get dependencies:
```
cd ~/turtlebot3_ws/src
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```
Build packages
```
cd ~/turtlebot3_ws/

source /opt/ros/humble/setup.bash

colcon build
```
Include the following lines in ~/.bashrc:
```
source /opt/ros/humble/local_setup.bash
source ~/turtlebot3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
```
## How to run
[**Turtlebot3**] \
Run the bringup
```
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30

source /opt/ros/humble/setup.bash

ros2 launch turtlebot3_bringup robot.launch.py
```

[**Remote PC Terminal 1**] \
Execute the launch file (Opens Rviz, Cartographer, Nav2 and exploration servers):
```
source /opt/ros/humble/local_setup.bash
source ~/turtlebot3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30

ros2 launch explorer_bringup explorer.launch.py
```
[**Remote PC Terminal 2**] \
Execute manager node and select the desired exploring algorithm:
```
source /opt/ros/humble/local_setup.bash
source ~/turtlebot3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=burger

ros2 run explorer_bringup manager
```
## Differences between real robot and simulation
- LDS-02, Number of values from LIDAR: Simulation: 360, real robot: ~222 --> the ranges had to be adjusted
## ToDo's:
- Watchtower does not know whether the exploration is completed or not --> Implement algorithm to calculate exploration status