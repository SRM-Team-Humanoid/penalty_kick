#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from penalty_kick.msg import Obstacle

#-------States---------------

ball_found = False
ball_center = False
ball_reached = False


#-----Params-----------------

tilt_thresh = 90
up_scan = 0
down_scan = 50








def ballUpdater(msg):
    global ball_coord,ball_found,ball_center

def ObsUpdater(msg):
    pass



if __name__ == '__main__':
    rospy.init_node('Core', anonymous=True)
    rospy.Subscriber("obstacles", Obstacle, ballUpdater)
    rospy.Subscriber("ball", Obstacle, appendObstacles)
    motionPublisher = rospy.Publisher('moco', String, queue_size=10)
    rospy.spin()