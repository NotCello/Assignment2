#! /usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
from assignment2_rt.msg import PlanningAction, PlanningGoal, Robot_info
from nav_msgs.msg import Odometry

# Callback function to process odometry data
def process_odometry(data):
    global robot_status_publisher

    robot_state = Robot_info()
    robot_state.x = data.pose.pose.position.x
    robot_state.y = data.pose.pose.position.y
    robot_state.vel_x = data.twist.twist.linear.x
    robot_state.vel_z = data.twist.twist.angular.z

    robot_status_publisher.publish(robot_state)

def on_feedback(feedback):
    if feedback.stat == "Target reached!":
        rospy.loginfo("Target reached!\nPosition: {}\nStatus: {}".format(feedback.actual_pose, feedback.stat))
        print("Options: (g=goal, x=cancel, e=exit)")

def send_target(client, goal_x, goal_y):
    goal_msg = PlanningGoal()
    goal_msg.target_pose.pose.position.x = goal_x
    goal_msg.target_pose.pose.position.y = goal_y

    client.send_goal(goal_msg, feedback_cb=on_feedback)
    rospy.loginfo("Goal sent: x={}, y={}".format(goal_x, goal_y))

def cancel_target(client):
    if client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
        rospy.loginfo("Cancelling active goal")
        client.cancel_goal()
        rospy.sleep(0.5)
        state = client.get_state()
        if state in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
            rospy.loginfo("Goal successfully cancelled")
        else:
            rospy.logwarn("Failed to cancel the goal")
    else:
        rospy.logwarn("No active goal to cancel")

def get_coordinates():
    while True:
        try:
            x_coord = float(input("Enter x coordinate: "))
            y_coord = float(input("Enter y coordinate: "))
            return x_coord, y_coord
        except ValueError:
            rospy.logwarn("Invalid input, please enter numeric values")

def main():
    global robot_status_publisher

    rospy.init_node('goal_manager')

    # Publisher for robot status
    robot_status_publisher = rospy.Publisher('/robot_status', Robot_info, queue_size=10)

    # Subscriber for odometry data
    rospy.Subscriber('/odom', Odometry, process_odometry)
    rospy.wait_for_message('/odom', Odometry)

    # Action client for sending goals
    action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for action server...")
    action_client.wait_for_server()

    while not rospy.is_shutdown():
        rospy.loginfo_once("Commands: 'g' to set a goal, 'x' to cancel, 'e' to exit")
        command = input("Enter command (g=goal, x=cancel, e=exit): ").strip().lower()

        if command == 'g':
            x, y = get_coordinates()
            send_target(action_client, x, y)
        elif command == 'x':
            cancel_target(action_client)
        elif command == 'e':
            rospy.loginfo("Exiting goal manager")
            break
        else:
            rospy.logwarn("Invalid command, try again")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")

