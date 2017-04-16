#!/usr/bin/env python
import rospy
from penalty_kick.msg import Obstacle
from std_msgs.msg import String
from ImagePublisher import GetSingleObstacle
import operator

#-----Params-----------------

tilt_thresh = 90
up_scan = 0
down_scan = 50

#-----------------------------

class Ball():
    def __init__(self):
        self.found = False
        self.center = False
        self.reached = False
        self.x = None
        self.y = None

    def update(self,ball):
        if ball==None:
            self.found = False
            self.center = False
            self.reached = False
            self.x = None
            self.y = None
        else:
            self.found = True
            self.x = ball[0]
            self.y = ball[1]
            if self.x in range(210,420) and self.y in range(160,320):
                self.center = True
            else:
                self.center = False
            #To Implement Tilt Thresh Here <<<<<<<<<<<<<<<<<<<<<<<<<<<<<------


class GetMultipleObstacles():
    def __init__(self):
        self.obstacles_list = []
    def getObjectsByColors(self, objectColor):
        return [ob for ob in self.obstacles_list if ob.color == objectColor]

    def getBall(self):
        Balls = self.getObjectsByColors('yellow')
        if len(Balls) != 0:
            xcen = Balls[0].x+Balls[0].w/2
            ycen = Balls[0].y+Balls[0].h/2
            return xcen,ycen
        else:
            return None

def detectBall():
    global MultipleObstacles
    Ball = MultipleObstacles.getBall()
    if Ball != None:
        print Ball


def appendObstacles(msg):
    global prevFrame, MultipleObstacles, BufferObstacles,ball
    SingleObstacle = GetSingleObstacle(msg.x, msg.y, msg.w, msg.h, msg.color)
    if msg.frame != prevFrame:
        if BufferObstacles != None:
            MultipleObstacles = BufferObstacles
            BufferObstacles = GetMultipleObstacles()
    BufferObstacles.obstacles_list.append(SingleObstacle)
    ball.update(MultipleObstacles.getBall())
    prevFrame = msg.frame



ball = Ball()


if __name__ == '__main__':
    prevFrame = 0
    MultipleObstacles = GetMultipleObstacles()
    BufferObstacles = GetMultipleObstacles()
    rospy.init_node('Intermediate', anonymous=True)
    rospy.Subscriber("obstacles", Obstacle, appendObstacles)
    moco = rospy.Publisher('moco', String, queue_size=1)
    while True:
        while not ball.found:
            moco.publish('none')
            pass #Temporary
        while not ball.center and ball.x!=None:
            if ball.x < 210:
                print ball.x
                moco.publish('hr')
            elif ball.x > 420:
                print ball.x
                moco.publish('hl')
            elif ball.y < 160:
                moco.publish('hu')
            elif ball.y > 320:
                moco.publish('hd')
        moco.publish('none')

    rospy.spin()
