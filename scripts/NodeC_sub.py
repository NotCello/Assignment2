#! /usr/bin/env python 

## \package assignment_2_2024
#  
#  \file NodeC_sub.py
#  \brief A ROS service node for retrieving the latest goal set for the robot
#  
#  \author Your Name
#  \version 1.0
#  \date 12/03/2025
#  
#  \details
#  This service node keeps track of the most recent goal sent to the robot.
#  It subscribes to the /reaching_goal/goal topic to store the latest goal,
#  and provides a service at /retrieve_latest_goal to return the last goal set.
#  
#  **Subscriber:**
#  - /reaching_goal/goal
#  
#  **Service:**
#  - /retrieve_latest_goal

import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment_2_2024.srv import Target, TargetResponse

##
#  \brief Stores the most recent goal received
#  
#  This global variable keeps track of the latest goal set for the robot.
latest_goal = None 

##
#  \brief Callback function to save the most recent goal
#  
#  \param message The PlanningActionGoal message received from the /reaching_goal/goal topic
#  
#  Logs the new goal received and updates the global latest_goal variable.
def save_goal_callback(message):
    global latest_goal
    latest_goal = message
    rospy.loginfo(f"New goal received: {latest_goal.goal.target_pose}")

##
#  \brief Service handler to return the most recent goal
#  
#  \param request The incoming service request (unused in this case)
#  \return TargetResponse containing the latest goal's target pose
#  
#  If no goal has been set yet, logs a warning and returns an empty response.
def process_goal_request(request):
    global latest_goal
    if latest_goal is None:
        rospy.logwarn("[SERVICE NODE] No goal has been set yet")
        return TargetResponse()
    goal_data = latest_goal.goal.target_pose
    return TargetResponse(goal_data)

##
#  \brief Initializes and runs the goal service node
#  
#  Sets up the ROS subscriber and service, and starts listening for incoming requests.
def goal_service_node():
    rospy.init_node('latest_goal_retriever')
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, save_goal_callback)
    rospy.Service('/retrieve_latest_goal', Target, process_goal_request)
    rospy.loginfo("Goal service node is now running and awaiting requests...")
    rospy.spin()

if __name__ == "__main__":
    try:
        goal_service_node()
    except rospy.ROSInterruptException:
        print("Service node terminated")

