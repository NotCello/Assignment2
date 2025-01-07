#! /usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
import assignment_2_2024.msg
from assignment_2_2024.msg import RobotInfo
from nav_msgs.msg import Odometry

# Callback for retrieving robot state from the topic /odom
def odometry_callback(data):
    global state_publisher

    robot_state = RobotInfo()
    robot_state.coord_x = data.pose.pose.position.x
    robot_state.coord_y = data.pose.pose.position.y
    robot_state.linear_speed = data.twist.twist.linear.x
    robot_state.angular_speed = data.twist.twist.angular.z

    state_publisher.publish(robot_state)

def feedback_handler(feedback):
    if feedback.stat == "Target reached!":
        print("")
        rospy.logwarn("Target reached at position\n{}\nStatus: {}".format(feedback.actual_pose, feedback.stat))
        print("Command (s=set goal, c=cancel goal, q=quit): ")

def dispatch_goal(action_client, x_target, y_target):
    # Prepare a goal message to send to the action server.
    goal_msg = assignment_2_2024.msg.PlanningGoal()
    goal_msg.target_pose.pose.position.x = x_target
    goal_msg.target_pose.pose.position.y = y_target

    goal_msg.target_pose.pose.position.z = 0.0      # Not supported by this robot
    goal_msg.target_pose.pose.orientation.x = 0.0   # Default values for unsupported fields
    goal_msg.target_pose.pose.orientation.y = 0.0
    goal_msg.target_pose.pose.orientation.z = 0.0
    goal_msg.target_pose.pose.orientation.w = 0.0

    # Send the goal to the action server.
    action_client.send_goal(goal_msg, feedback_cb=feedback_handler)
    rospy.loginfo("Goal dispatched")

def abort_goal(action_client):
    if action_client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
        rospy.loginfo("Attempting to cancel the active goal")
        action_client.cancel_goal()
        rospy.sleep(0.5)  # Allow time for the cancellation to propagate
        current_state = action_client.get_state()
        if current_state in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
            rospy.loginfo("Goal successfully cancelled")
        else:
            rospy.logwarn("Failed to cancel the goal")
    else:
        rospy.logwarn("No active goal to cancel.")

def retrieve_coordinates():
    while True:
        try:
            x_val = float(input("Enter the x coordinate: "))
            y_val = float(input("Enter the y coordinate: "))
            return x_val, y_val
        except ValueError:
            rospy.logwarn("Invalid input. Please enter numerical values only.")

def main():
    global state_publisher
    rospy.init_node('robot_action_client')

    # Create the publisher for robot state information
    state_publisher = rospy.Publisher('/robot_status', RobotInfo, queue_size=10)
    # Wait for an initial message from the /odom topic
    rospy.wait_for_message('/odom', Odometry)
    # Subscribe to /odom to get updates on the robot's state
    rospy.Subscriber('/odom', Odometry, odometry_callback)

    # Set up the action client to interact with the action server
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
        print("Action client terminated.")

