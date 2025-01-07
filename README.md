# README for Assignment 2 Package

## Overview
This package, `assignment_2_2024`, is designed to work with ROS (Robot Operating System) to manage robot goals and retrieve the most recently set goal. It includes two main nodes:

1. **Node A**: A node that retrieves and publishes robot state information from the `/odom` topic and interacts with an action server to send and cancel goals.
2. **Node B**: A service node that retrieves the most recent goal set through the `/reaching_goal/goal` topic and provides it to clients via a ROS service.

This package demonstrates the integration of ROS publishers, subscribers, services, and action clients in a single system.

---

## Node A: Action Client

### Functionality
Node A subscribes to the `/odom` topic to retrieve the robot's current state (position and velocity) and publishes it to the `/robot_status` topic. It also acts as a client to an action server, allowing users to set new goals, cancel active goals, and monitor the feedback from the action server.

### Key Features
- Subscribes to `/odom` to retrieve robot state (position and velocity).
- Publishes robot state to `/robot_status`.
- Connects to an action server (`/reaching_goal`) to send, cancel, and monitor goals.
- Accepts user input via the terminal to set goals, cancel goals, or quit.

### How to Use
1. Launch the ROS core:
   ```bash
   roscore
   ```

2. Run the Node A script:
   ```bash
   rosrun assignment_2_2024 nodeA_client.py
   ```

3. Follow the on-screen prompts to:
   - Set a goal (`s`) by entering x and y coordinates.
   - Cancel the current goal (`c`).
   - Quit the program (`q`).

---

## Node B: Service Node

### Functionality
Node B listens to the `/reaching_goal/goal` topic to monitor the most recently set goal. It provides a ROS service (`/retrieve_latest_goal`) that allows clients to request the latest goal set. If no goal has been set yet, the service will log a warning and return an empty response.

### Key Features
- Subscribes to `/reaching_goal/goal` to track the most recent goal.
- Provides a service (`/retrieve_latest_goal`) to retrieve the latest goal.
- Logs a warning if no goal has been set when the service is called.

### How to Use
1. Launch the ROS core:
   ```bash
   roscore
   ```

2. Run the Node B script:
   ```bash
   rosrun assignment_2_2024 NodeB.py
   ```

3. Call the service to retrieve the latest goal:
   ```bash
   rosservice call /retrieve_latest_goal
   ```
   - If a goal has been set, it will return the goal's `PoseStamped` message.
   - If no goal has been set, it will log a warning.

---

## Installation and Compilation

### Prerequisites
- ROS Noetic or later installed on your system.
- A catkin workspace set up (e.g., `/root/my_ros_ws`).

### Clone the Repository
Navigate to the `src` directory of your catkin workspace and clone the repository:

```bash
cd ~/my_ros_ws/src
git clone https://github.com/<your-username>/assignment_2_2024.git
```

### Build the Package
After cloning, compile the package:

```bash
cd ~/my_ros_ws
catkin_make
```

### Source the Workspace
Source the workspace setup file to update your ROS environment:

```bash
source ~/my_ros_ws/devel/setup.bash
```

---

## How to Run
1. Start the ROS core:
   ```bash
   roscore
   ```

2. Launch Node A to set and manage goals:
   ```bash
   rosrun assignment_2_2024 nodeA_client.py
   ```

3. Launch Node B to retrieve the latest goal:
   ```bash
   rosrun assignment_2_2024 NodeB.py
   ```

4. Use `rosservice` commands to interact with Node B's service, or follow the prompts in Node A's terminal for goal management.

---

## Additional Notes
- Ensure the action server (`/reaching_goal`) is running for Node A to function correctly.
- Use `rostopic echo` to monitor the topics `/reaching_goal/goal` and `/robot_status` for debugging.
- Modify the code as needed to suit your specific robot and application.

---
