# README for Robot Movement Project (ROS 2)

## Overview
This project consists of a complete ROS 2 package designed to control and monitor a robot's movement. The package integrates:

1. **A Robot Movement Controller Node:** Allows the user to input velocities for the robot to move.
2. **An Auxiliary Service Node:** Enables the retrieval of the most recent goal set by the user.

This package demonstrates key concepts of ROS 2, including publishers, subscribers, and services, and can be used in both simulation and real-world robotic applications.

---

## Node 1: Robot Movement Controller

### Functionality
This node enables the user to manually control a robot's movement by specifying linear and angular velocities through terminal input. It publishes these velocity commands to the `/cmd_vel` topic, which is standard in ROS 2 for robot control.

### Key Features
- Publishes velocity commands (`Twist` messages) to the `/cmd_vel` topic.
- Restricts linear velocity to a configurable maximum threshold (`LINEAR_THR`).
- Automatically stops the robot after 3 seconds of movement.

### Node Details
- **Node Name:** `move_robot_node`
- **Published Topic:** `/cmd_vel` (message type: `geometry_msgs/Twist`)

### How to Use
1. **Run the Node**
   ```bash
   ros2 run <your_package_name> move_robot.py
   ```

2. **Input Commands**
   - Enter a linear velocity (`x`) value.
   - Enter an angular velocity (`z`) value.

3. **Behavior**
   - The robot moves for 3 seconds based on the provided inputs and then stops.
   - Users can continue to input new commands.

---

## Node 2: Retrieve Latest Goal

### Functionality
This service node tracks the most recent goal set for the robot's movement and provides it to users upon request. The node subscribes to the `/reaching_goal/goal` topic and offers a ROS service, `/retrieve_latest_goal`, to return the last set goal.

### Key Features
- Subscribes to `/reaching_goal/goal` to monitor new goals.
- Provides a ROS service (`/retrieve_latest_goal`) to retrieve the latest goal.
- Handles cases where no goal has been set by logging warnings.

### Node Details
- **Node Name:** `latest_goal_retriever`
- **Subscribed Topic:** `/reaching_goal/goal` (message type: `PlanningActionGoal`)
- **Service Provided:** `/retrieve_latest_goal` (service type: `Target`)

### How to Use
1. **Run the Node**
   ```bash
   ros2 run <your_package_name> goal_service_node.py
   ```

2. **Call the Service**
   Use the ROS 2 service tools to request the latest goal:
   ```bash
   ros2 service call /retrieve_latest_goal <your_package_name>/srv/Target
   ```

3. **Expected Behavior**
   - If a goal has been set, the service returns the goal's `PoseStamped` data.
   - If no goal has been set, a warning is logged, and an empty response is returned.

---

## Installation and Compilation

### Prerequisites
- ROS 2 (e.g., Humble, Foxy, or Galactic) installed on your system.
- A ROS 2 workspace set up (e.g., `~/ros2_ws`).

### Clone the Repository
Navigate to the `src` directory of your ROS 2 workspace and clone the repository:

```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/robot_movement_project.git
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

## Example Workflow
1. **Start the ROS 2 Core**
   ```bash
   ros2 daemon start
   ```

2. **Run the Movement Controller Node**
   ```bash
   ros2 run <your_package_name> move_robot.py
   ```

3. **Run the Goal Retrieval Node**
   ```bash
   ros2 run <your_package_name> goal_service_node.py
   ```

4. **Set a Goal and Retrieve It**
   - Use the Movement Controller to set goals.
   - Call the `/retrieve_latest_goal` service to fetch the latest goal set.

---

## Additional Notes
- Ensure your robot or simulation environment is set up to interpret commands from the `/cmd_vel` topic.
- Use a simulation tool like Gazebo or RViz if no physical robot is available.
- Modify parameters like `LINEAR_THR` in the code to suit your robot's capabilities.

---
