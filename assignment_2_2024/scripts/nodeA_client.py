#! /usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
import assignment_2_2024.msg
from assignment_2_2024.msg import Robot_info
from nav_msgs.msg import Odometry

# Callback for retrieve position and velocity from the topic /odom
def odom_callback(msg):
    global pub_position_vel
    
    robot_info = Robot_info()
    robot_info.x = msg.pose.pose.position.x
    robot_info.y = msg.pose.pose.position.y
    robot_info.vx = msg.twist.twist.linear.x
    robot_info.vz = msg.twist.twist.angular.z

    pub_position_vel.publish(robot_info)

def feedback_callback(feedback):
    if (feedback.stat == "Target reached!"):
        print("")
        rospy.logwarn("Target reached\n{}\nStatus: {}\n".format(feedback.actual_pose, feedback.stat))
        print("Command (s=set goal, c=cancel goal, q=quit): ")

def send_goal(client, x, y):    
    # Creates a goal to send to the action server.
    goal = assignment_2_2024.msg.PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    goal.target_pose.pose.position.z = 0.0      # Not supported by two wheels's robot
    goal.target_pose.pose.orientation.x = 0.0   # Not supported by the action server
    goal.target_pose.pose.orientation.y = 0.0   # Not supported by the action server
    goal.target_pose.pose.orientation.z = 0.0   # Not supported by the action server
    goal.target_pose.pose.orientation.w = 0.0   # Not supported by the action server

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_callback)
    rospy.loginfo("Goal sent")
    
    # Waits for the server to finish performing the action.
    #client.wait_for_result()

    # Prints out the result of executing the action
    #return client.get_result()
    
def cancel_goal(client):
    if client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
        rospy.loginfo("Cancelling current goal")
        client.cancel_goal()
        rospy.sleep(0.5)  # Timeout for forward the cancel
        state = client.get_state()
        if state in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
            rospy.loginfo("Goal successfully cancelled")
        else:
            rospy.logwarn("Failed to cancel the goal")
    else:
        rospy.logwarn("No active goal to cancel.")

def get_input():
    while True:
        try:
            x = float(input("Enter the value for setting the x coordinate: "))
            y = float(input("Enter the value for setting the y coordinate: "))
            return x, y
        except ValueError:
            rospy.logwarn("Input not valid, enter only numbers!")

def main():
    global pub_position_vel
    rospy.init_node('action_client')

    # Create the publisher for postion and velocity
    pub_position_vel = rospy.Publisher('/robot_information', Robot_info, queue_size=10)
    # Wait for the arrival of the first message on that topic
    rospy.wait_for_message('/odom', Odometry)
    # Subscribe to /odom topic for retrieve the data from the robot
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    # Creates the SimpleActionClient, passing the type of the action (PlanningAction) to the constructor
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2024.msg.PlanningAction)
    rospy.loginfo("Waiting that the action server is avaible")
    client.wait_for_server()
    
    while not rospy.is_shutdown():
        rospy.loginfo_once("Enter 's' to set a goal, 'c' to cancel the current goal, 'q' to quit the action client or 'CTRL+C' for close all the simulation")
        answer = input("Command (s=set goal, c=cancel goal, q=quit, CTRL+C=exit all): ").strip().lower()
        if answer == 's':
            x, y = get_input()
            send_goal(client, x, y)
        elif answer == 'c':
            cancel_goal(client)
        elif answer == 'q':
            rospy.loginfo("Exiting from the action client")
            break
        else:
            rospy.logwarn("Invalid command. Please try again")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted")
