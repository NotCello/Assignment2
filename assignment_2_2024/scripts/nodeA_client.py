assign#!/usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
from assignment2_rt.msg import PlanningAction, PlanningGoal, RobotInfo
from nav_msgs.msg import Odometry

def odometry_callback(data):
    global robot_info_publisher

    robot_status = RobotInfo()
    robot_status.position_x = data.pose.pose.position.x
    robot_status.position_y = data.pose.pose.position.y
    robot_status.linear_velocity = data.twist.twist.linear.x
    robot_status.angular_velocity = data.twist.twist.angular.z

    robot_info_publisher.publish(robot_status)

def feedback_handler(feedback):
    if feedback.status == "Target achieved!":
        rospy.loginfo("Target achieved at position: {}\nStatus: {}".format(feedback.current_pose, feedback.status))
        print("Options: (g=goal, x=cancel, e=exit)")

def dispatch_goal(client, x_coord, y_coord):
    goal_message = PlanningGoal()
    goal_message.target_pose.pose.position.x = x_coord
    goal_message.target_pose.pose.position.y = y_coord

    client.send_goal(goal_message, feedback_cb=feedback_handler)
    rospy.loginfo("Dispatched goal to coordinates: x={}, y={}".format(x_coord, y_coord))

def abort_goal(client):
    if client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
        rospy.loginfo("Aborting active goal")
        client.cancel_goal()
        rospy.sleep(0.5)
        state = client.get_state()
        if state in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
            rospy.loginfo("Goal successfully aborted")
        else:
            rospy.logwarn("Failed to abort the goal")
    else:
        rospy.logwarn("No active goal to abort")

def acquire_coordinates():
    while True:
        try:
            x_input = float(input("Enter x coordinate: "))
            y_input = float(input("Enter y coordinate: "))
            return x_input, y_input
        except ValueError:
            rospy.logwarn("Invalid input, please enter numeric values")

def main():
    global robot_info_publisher

    rospy.init_node('goal_manager_node')

    robot_info_publisher = rospy.Publisher('/robot_status', RobotInfo, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.wait_for_message('/odom', Odometry)

    action_client = actionlib.SimpleActionClient('/reach_goal', PlanningAction)
    rospy.loginfo("Awaiting action server...")
    action_client.wait_for_server()

    while not rospy.is_shutdown():
        rospy.loginfo_once("Commands: 'g' to set a goal, 'x' to cancel, 'e' to exit")
        user_command = input("Enter command (g=goal, x=cancel, e=exit): ").strip().lower()

        if user_command == 'g':
            x, y = acquire_coordinates()
            dispatch_goal(action_client, x, y)
        elif user_command == 'x':
            abort_goal(action_client)
        elif user_command == 'e':
            rospy.loginfo("Exiting goal manager node")
            break
        else:
            rospy.logwarn("Invalid command, please try again")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")

