#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def swapper(data):
    marco.publish( data.data)

marco = rospy.Publisher('swapPublish', String, queue_size=1)
rospy.init_node('swap', anonymous=True)
rospy.Subscriber('swapSubscriber',String,swapper)