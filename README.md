# README for Robot Navigation Package

## Overview
This repository contains two branches, each implementing different robot navigation functionalities using ROS:

1. **ROS 1 Branch:** Includes an action client and a service node to handle robot goals.
2. **ROS 2 Branch:** Provides a node for robot navigation, allowing user-defined velocity commands in a simulation environment.

---

## Branch Descriptions

### ROS 1 Branch
The ROS 1 implementation offers the following:

#### **Action Client Node**
- Enables the user to set or cancel target positions (x, y).
- Utilizes feedback from the action server to track when the robot reaches its target.
- Publishes the robot's state (position and velocity) as a custom message (`x`, `y`, `vel_x`, `vel_z`) based on the `/odom` topic.

#### **Service Node**
- Provides the last set target position when called.

#### **Launch File**
- Includes a complete simulation setup to run all nodes seamlessly.

### ROS 2 Branch
The ROS 2 implementation focuses on:

#### **Robot Movement Node**
- Allows users to control the robot's motion by inputting linear and angular velocities via the terminal.
- The robot moves for 3 seconds per input and stops automatically, awaiting further commands.
- Publishes velocity commands to the `/cmd_vel` topic.

---

## Getting Started

### Clone the Repository
Download the repository to your ROS workspace:

```bash
cd ~/ros_workspace/src
git clone https://github.com/<your-username>/robot_navigation_package.git
```

### Switch to the Desired Branch
To use the ROS 1 or ROS 2 implementation, checkout the respective branch:

- For ROS 1:
  ```bash
  git checkout ros1
  ```

- For ROS 2:
  ```bash
  git checkout ros2
  ```

---

## Installation and Setup

### ROS 1
1. **Build the Package:**
   ```bash
   cd ~/ros_workspace
   catkin_make
   source devel/setup.bash
   ```

2. **Run the Nodes:**
   - Action Client:
     ```bash
     rosrun robot_navigation_package nodeA_client.py
     ```
   - Service Node:
     ```bash
     rosrun robot_navigation_package NodeB.py
     ```

3. **Launch Simulation:**
   ```bash
   roslaunch robot_navigation_package simulation.launch
   ```

### ROS 2
1. **Build the Package:**
   ```bash
   cd ~/ros_workspace
   colcon build
   source install/setup.bash
   ```

2. **Run the Node:**
   ```bash
   ros2 run robot_navigation_package move_robot.py
   ```

---

## Additional Information
- Use `rostopic echo` or `ros2 topic echo` to monitor relevant topics (e.g., `/cmd_vel`, `/reaching_goal/goal`).
- Both branches include README files with specific instructions for usage and testing.
- The package is compatible with both simulated and real robot environments.

---
