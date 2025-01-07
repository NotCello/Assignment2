#!/usr/bin/env python

import rospy
from assignment_2_2024.srv import Target, TargetResponse
from geometry_msgs.msg import PoseStamped

# Callback del servizio
def service_callback(request):
    rospy.loginfo("Service request received")
    response = TargetResponse()
    response.last_target = PoseStamped()

    # Imposta i valori di risposta
    response.last_target.header.stamp = rospy.Time.now()
    response.last_target.header.frame_id = "map"
    response.last_target.pose.position.x = 1.0  # Modifica con valori reali se necessario
    response.last_target.pose.position.y = 2.0
    response.last_target.pose.position.z = 0.0
    response.last_target.pose.orientation.x = 0.0
    response.last_target.pose.orientation.y = 0.0
    response.last_target.pose.orientation.z = 0.0
    response.last_target.pose.orientation.w = 1.0

    rospy.loginfo("Returning last target")
    return response

# Funzione principale
def main():
    rospy.init_node('custom_service_server')

    # Crea il server del servizio
    service = rospy.Service('/custom_service', Target, service_callback)
    rospy.loginfo("Service '/custom_service' is ready and awaiting requests.")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Service server node interrupted")

