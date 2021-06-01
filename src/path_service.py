#!/usr/bin/env python3

# ------------PROGRAM ZA SERVIS ZA UNOSENJE PUTANJE ROBOTA------------
#
import rospy
from ROS_TurtleBot3_Localization_with_Extended_Kalman_Filter.srv import path, pathResponse
from std_msgs.msg import String

# -------------NOD I PUBLISHER NA TOPIC '/path'--------------
#
rospy.init_node('path', anonymous=False)
pub = rospy.Publisher('path', String, queue_size=1)

# -------------CALLBACK FUNKCIJA ZA SERVISE--------------
#
def response_callback(req):

    pub.publish(req.path)

    return pathResponse(True)

# -------------FORMIRANJE ROS SERVISA ZA UNOSENJE PUTANJE ROBOTA--------------
#
s = rospy.Service('path', path, response_callback)
rospy.loginfo('Servis za zadavanje putanje robota je spreman!')
rospy.spin()