#! /usr/bin/env python 

import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment_2_2024.srv import Target, TargetResponse

latest_goal = None # Store the most recent goal

# Callback to save the most recent goal
def save_goal_callback(message):
    global latest_goal
    latest_goal = message
    rospy.loginfo(f"New goal received: {latest_goal.goal.target_pose}")

# Service handler to return the most recent goal
def process_goal_request(request):
    global latest_goal
    if latest_goal is None:
        rospy.logwarn("[SERVICE NODE] No goal has been set yet")
        return TargetResponse()
    goal_data = latest_goal.goal.target_pose
    return TargetResponse(goal_data)

# Initialize and run the service node
def goal_service_node():
    rospy.init_node('latest_goal_retriever')
    # Subscriber to monitor new goals
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, save_goal_callback)
    # Service to provide the most recent goal
    rospy.Service('/retrieve_latest_goal', Target, process_goal_request)
    rospy.loginfo("Goal service node is now running and awaiting requests...")
    rospy.spin()

if __name__ == "__main__":
    try:
        goal_service_node()
    except rospy.ROSInterruptException:
        print("Service node terminated")

