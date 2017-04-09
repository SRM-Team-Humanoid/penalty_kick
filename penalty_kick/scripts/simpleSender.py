#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def moco(data):
    cmdDeg = data.data
    print cmdDeg

marco = rospy.Publisher('sender', String, queue_size=1)
rospy.init_node('Intermediate')
rospy.Subscriber('feedback',String,moco)
while True:
    marco.publish('lol')
