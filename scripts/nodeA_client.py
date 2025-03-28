#! /usr/bin/env python

## @package assignment_2_2024
# 
#  \file nodeA_client.py
#  \brief A ROS action client for sending movement goals to the robot.
#  
#  \author Marcello Ori
#  \version 1.0
#  \date 12/03/2025
#  
#  \details
#  This script implements a ROS action client that allows users to set movement goals
#  for the robot. It continuously retrieves the robot's current state from the 
#  `/odom` topic and publishes updates to the `/robot_status` topic. The user can:
#  - Set a movement goal through input
#  - Cancel an active goal if needed
#  - Exit the program gracefully
#
#  **Publisher:**
#  - `/robot_status`
#  
#  **Subscriber:**
#  - `/odom`
#  
#  **Action Client:**
#  - `/reaching_goal`

import rospy
import actionlib
from actionlib import GoalStatus
import assignment_2_2024.msg
from assignment_2_2024.msg import RobotInfo
from nav_msgs.msg import Odometry

##
#  \brief Callback function that processes odometry data.
#  
#  \param data The Odometry message containing the robot's current state.
#  
#  This function retrieves the robot's position and speed from the odometry 
#  data and publishes this information to the `/robot_status` topic.
def odometry_callback(data):
    global state_publisher
    robot_state = RobotInfo()
    
    robot_state.coord_x = data.pose.pose.position.x
    robot_state.coord_y = data.pose.pose.position.y
    robot_state.linear_speed = data.twist.twist.linear.x
    robot_state.angular_speed = data.twist.twist.angular.z
    
    state_publisher.publish(robot_state)

##
#  \brief Handles feedback from the action server.
#  
#  \param feedback The feedback message from the action server.
#  
#  This function logs a message when the robot successfully reaches 
#  the target position.
def feedback_handler(feedback):
    if feedback.stat == "Target reached!":
        rospy.logwarn("Target reached at position\n{}\nStatus: {}".format(feedback.actual_pose, feedback.stat))
        print("Command (s=set goal, c=cancel goal, q=quit): ")

##
#  \brief Sends a movement goal to the action server.
#  
#  \param action_client The instance of the action client used for communication.
#  \param x_target Target x coordinate for movement.
#  \param y_target Target y coordinate for movement.
#  
#  Constructs and sends the goal to the `/reaching_goal` action server. 
#  The goal's orientation is set to default values.
def dispatch_goal(action_client, x_target, y_target):
    goal_msg = assignment_2_2024.msg.PlanningGoal()
    
    goal_msg.target_pose.pose.position.x = x_target
    goal_msg.target_pose.pose.position.y = y_target
    goal_msg.target_pose.pose.position.z = 0.0  # Assumes ground level
    goal_msg.target_pose.pose.orientation.x = 0.0
    goal_msg.target_pose.pose.orientation.y = 0.0
    goal_msg.target_pose.pose.orientation.z = 0.0
    goal_msg.target_pose.pose.orientation.w = 1.0  # Default valid quaternion
    
    action_client.send_goal(goal_msg, feedback_cb=feedback_handler)
    rospy.loginfo("Goal dispatched")

##
#  \brief Attempts to cancel an active movement goal.
#  
#  \param action_client The instance of the action client involved in the goal.
#  
#  Cancels the active goal if it exists and logs the result of the operation.
def abort_goal(action_client):
    if action_client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
        rospy.loginfo("Attempting to cancel the active goal")
        action_client.cancel_goal()
        rospy.sleep(0.5)

        current_state = action_client.get_state()
        if current_state in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
            rospy.loginfo("Goal successfully cancelled")
        else:
            rospy.logwarn("Failed to cancel the goal")
    else:
        rospy.logwarn("No active goal to cancel.")

##
#  \brief Retrieves target coordinates from user input.
#  
#  \return Tuple (x_val, y_val) containing the target x and y coordinates.
#  
#  Ensures that only valid numerical inputs are accepted.
def retrieve_coordinates():
    while True:
        try:
            x_val = float(input("Enter the x coordinate: "))
            y_val = float(input("Enter the y coordinate: "))
            return x_val, y_val
        except ValueError:
            rospy.logwarn("Invalid input. Please enter numerical values only.")

##
#  \brief The main function of the action client script.
#  
#  Initializes the ROS node, sets up the action client, and manages user commands 
#  to control the robot's movements.
def main():
    global state_publisher
    rospy.init_node('robot_action_client')
    
    state_publisher = rospy.Publisher('/robot_status', RobotInfo, queue_size=10)
    rospy.wait_for_message('/odom', Odometry)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    
    action_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2024.msg.PlanningAction)
    rospy.loginfo("Waiting for the action server to become available...")
    action_client.wait_for_server()

    while not rospy.is_shutdown():
        rospy.loginfo_once("Use 's' to set a goal, 'c' to cancel the goal, 'q' to quit, or CTRL+C to terminate.")
        user_input = input("Command (s=set goal, c=cancel goal, q=quit): ").strip().lower()

        if user_input == 's':
            x_target, y_target = retrieve_coordinates()
            dispatch_goal(action_client, x_target, y_target)
        elif user_input == 'c':
            abort_goal(action_client)
        elif user_input == 'q':
            rospy.loginfo("Exiting the action client")
            break
        else:
            rospy.logwarn("Invalid command. Please try again.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action client terminated.")

