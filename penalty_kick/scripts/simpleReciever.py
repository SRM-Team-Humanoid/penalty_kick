#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def moco(data):
    cmdDeg = data.data
    print cmdDeg
rospy.init_node('Intermediate2')

marco = rospy.Publisher('feedback', String, queue_size=1)
rospy.Subscriber('sender',String,moco)

while True:
    marco.publish('none')