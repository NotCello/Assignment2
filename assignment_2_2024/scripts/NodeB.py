#! /usr/bin/env python

import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment_2_2024.srv import Target, TargetResponse

last_target = None # Save the last goal set

# Callback to save the last set goal
def goal_callback(msg):
    global last_target
    last_target = msg
    #rospy.loginfo(f"Received a new goal: {last_target.goal.target_pose}")

# Returns the last set goal to the service caller
def handle_last_goal_request(req):
    global last_target
    if last_target is None:
        rospy.logwarn("[SERVICE NODE] No target available")
        return TargetResponse()
    target_info = last_target.goal.target_pose
    return TargetResponse(target_info)
    
# 
def service_last_goal():
    rospy.init_node('retrieve_last_target')
    # Subscriber to receive the set goals
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)
    # Service that returns the last set goal
    rospy.Service('/get_last_goal', Target, handle_last_goal_request)
    rospy.loginfo("Service node started. Waiting for requests...")
    rospy.spin()

if __name__ == "__main__":
    try:
        service_last_goal()
    except rospy.ROSInterruptException:
        print("Program interrupted")
