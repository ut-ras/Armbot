# Armbot
Armbot Repository, contains ROS workspace and other code.

Current Contributors are Jacob, Shreeya, and Kevin


## Installation
1. Make the Ros2 Humble workspace (Install ROS2 first)
```bash
mkdir -p ~/armbot_ws/src
cd ~/armbot_ws
colcon build
```
2. Clone the repository
```bash
cd ~/armbot_ws/src 
git clone https://github.com/ut-ras/Armbot.git
```
3. Build the workspace
```bash
cd ~/armbot_ws
colcon build
```

## Folder Explanation
- **armbot_description**: Contains the URDF and meshes for the robot
- **armbot_gazebo**: Contains the gazebo world and launch files
- **armbot_control**: Contains the control code for the robot
- **armbot_moveit**: Contains the moveit configuration for the robot
- **armbot_bringup**: Contains the launch files to bring up the robot
- **armbot_msgs**: Contains the custom messages/services for the robot
