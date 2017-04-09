#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def vision_proc():
    return "Boom!"



def vision_pub():
    pub = rospy.Publisher('vision_data', String, queue_size=10)
    rospy.init_node('Vision', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = vision_proc()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        vision_pub()
    except rospy.ROSInterruptException:
        pass
