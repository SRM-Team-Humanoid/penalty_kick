#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def moco(data):
    cmdDeg = data.data
    while True:
        pass


def moco2(data):
    cmdDeg = data.data
    print cmdDeg

rospy.init_node('Intermediate2')


rospy.Subscriber('sender',String,moco)
rospy.Subscriber('sender2',String,moco2)

rospy.spin()