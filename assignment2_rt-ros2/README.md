# Assignment 2 (Part II) - Robot Navigation with ROS 2

## Overview
This project involves creating a ROS 2 package named `assignment2_rt` that enables controlling a robot in a simulation environment. The functionality allows users to set linear and angular velocities, moving the robot for 3 seconds per command until the program is terminated.

### Node Functionality
The program consists of a single node that interacts with the user through the terminal to control the robot's motion:
- **User Input**: At each step, the user is prompted to provide:
  - Linear velocity (`x`)
  - Angular velocity (`z`)
- **Robot Motion**: The robot moves with the specified velocities for 3 seconds.
- **Termination**: The program runs continuously until the user interrupts it.

The robot starts in the simulation environment at position `(2,2)`.

## Nodes Description

### Movement Node
This single node interacts with the user and commands the robot:
- **User Commands**:
  - Enter the desired linear velocity (`x`) and angular velocity (`z`).
- **Robot Behavior**:
  - Upon receiving input, the robot moves in the simulation environment with the given velocities for 3 seconds.

This node uses ROS 2 publisher to control the robot’s movement, publishing velocity data on the `/cmd_vel` topic.

## Installation

### Prerequisites
- **ROS 2** must be installed on your system
- A properly configured ROS 2 workspace
- Clone the package `robot_urdf` at the following link: [robot_urdf](https://github.com/CarmineD8/robot_urdf)
- Run this command: `sudo apt-get install ros-ROS2DISTRO-xacro ros-ROS2DISTRO-joint-state-publisher ros-ROS2DISTRO-gazebo*`

### 1. Clone the Package
- Open a terminal
- Navigate to the `src` folder of your ROS 2 workspace:
  ```bash
  cd ~/ros2_ws/src
  ```
- Clone the repository for the package:
  ```bash
  git clone https://github.com/AlessandroMangili/assignment2_rt
  ```
- Switch to the ROS 2 branch:
  ```bash
  cd assignment2_rt
  git switch ros2
  ```

### 2. Build the Package
- Navigate back to the root of your ROS 2 workspace:
  ```bash
  cd ~/ros2_ws
  ```
- Build the package:
  ```bash
  colcon build
  ```
- Verify the build process completes successfully.

### 3. Source the Environment
- Update your ROS 2 environment variables to include the new package. Add the following line to your `~/.bashrc` file:
  ```bash
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
  ```
  To apply the changes, run:
  ```bash
  source ~/.bashrc
  ```

## Run the Package

### 1. Launch the Simulation
- Open a terminal and launch the simulation robot environment
  ```bash
  ros2 launch robot_urdf gazebo.launch.py
  ```
  where `robot_urdf` is the package cloned before.

### 2. Run the Movement Node
- Run the node to control the robot:
  ```bash
  ros2 run assignment2_rt motion
  ```
- Follow the on-screen prompts to input the robot’s linear (`x`) and angular (`z`) velocities.

## Interacting with the Node

- **Enter Velocities**: Each time the node prompts, input the linear (`x`) and angular (`z`) velocities.
- **Movement Execution**: The robot will execute the motion for 3 seconds.
- **Continue or Exit**: Continue providing new velocities until you decide to terminate the program.
