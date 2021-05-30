#!/usr/bin/env python3

# Ova skripta predstavlja ROS servis kojim je implementiran
# unos komandi za pomeranje robota prilikom njegove lokalizacije

# Ucitavanje potrebnih biblioteka i struktura poruka
import rospy
from ROS_TurtleBot3_Localization_with_Extended_Kalman_Filter.srv import control, controlResponse
from std_msgs.msg import String

# Inicijalizacija globalnog publisher-a na topic /vel_control
# preko kojeg se komunicira sa ostalim skriptama o trenutnoj linearnoj i 
# ugaonoj brzini robota

rospy.init_node('robot_control', anonymous=False)
pub = rospy.Publisher('robot_control', String, queue_size=1)

# Callback funkcija za ROS servis
# koja publish-uje brzine do glavne skripte koje su zadate preko konzole
def response_callback(req):

    pub.publish(req.control)
    
    # Povratna vrednost servisa
    return controlResponse(True)

# Inicijalizacija ROS servisa za unos brzina robota
s = rospy.Service('control', control, response_callback)
rospy.loginfo("Servis za kontrolu robota je spreman!")
rospy.spin()