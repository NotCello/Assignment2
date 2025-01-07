# README for Robot Movement Node (ROS 2)

## Overview
This package provides a simple ROS 2 implementation to control a robot's movement using linear and angular velocities. The node subscribes to user inputs via the terminal and publishes velocity commands to the `/cmd_vel` topic. This package is designed for basic robot motion testing and simulation environments.

---

## Node: Robot Movement Controller

### Functionality
This node allows a user to control a robot's movement by entering linear and angular velocities through the terminal. The node publishes the velocity commands to the `/cmd_vel` topic, which is commonly used in ROS 2 for robot control.

### Key Features
- Publishes velocity commands (`Twist` messages) to the `/cmd_vel` topic.
- Restricts linear velocity to a maximum threshold of `5.0` (adjustable via `LINEAR_THR`).
- Allows the user to input linear and angular velocities via the terminal.
- Stops the robot automatically after 3 seconds of movement or upon user termination (Ctrl+C).

### Node Description
- **Node Name:** `move_robot_node`
- **Topic Published:** `/cmd_vel` (message type: `geometry_msgs/Twist`)

### How to Use
1. **Run the Node**
   ```bash
   ros2 run <your_package_name> move_robot.py
   ```

2. **Input Commands**
   - Enter a linear velocity (`x`) value.
   - Enter an angular velocity (`z`) value.

3. The robot will move for 3 seconds based on the input values and then stop. You can repeat the process by entering new values.

---

## Installation and Compilation

### Prerequisites
- ROS 2 (e.g., Humble, Foxy, or Galactic) installed on your system.
- A ROS 2 workspace set up (e.g., `~/ros2_ws`).

### Clone the Repository
Navigate to the `src` directory of your ROS 2 workspace and clone the repository:

```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/robot_movement_ros2.git
```

### Build the Package
After cloning the repository, build the package using the following commands:

```bash
cd ~/ros2_ws
colcon build
```

### Source the Workspace
Source the workspace to update your ROS 2 environment:

```bash
source install/setup.bash
```

---

## How to Run
1. **Launch the ROS 2 Core**
   Make sure the ROS 2 core is running:
   ```bash
   ros2 daemon start
   ```

2. **Run the Node**
   Execute the movement node:
   ```bash
   ros2 run <your_package_name> move_robot.py
   ```

3. **Input Velocities**
   Follow the prompts in the terminal to:
   - Set a linear velocity (x-direction).
   - Set an angular velocity (z-direction).

4. **Stop the Robot**
   The robot will stop automatically after 3 seconds of movement or if interrupted (Ctrl+C).

---

## Example
**Terminal Input:**
```
Enter the linear velocity x: 2.0
Enter angular velocity z: 0.5
```

**Expected Behavior:**
- The robot moves forward with a linear velocity of `2.0` and an angular velocity of `0.5` for 3 seconds.
- The robot stops after 3 seconds and prompts for new velocities.

---

## Additional Notes
- Ensure that the `/cmd_vel` topic is connected to a robot or simulation capable of interpreting velocity commands.
- Modify the `LINEAR_THR` value in the code to adjust the maximum allowed linear velocity.
- Use a simulation tool like Gazebo or RViz to test the node if no physical robot is available.

---
