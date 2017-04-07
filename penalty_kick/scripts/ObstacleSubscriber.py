#!/usr/bin/env python
import rospy
from obstacle_run.msg import Obstacle
from std_msgs.msg import String
from ImagePublisher import GetSingleObstacle
import operator

class GetMultipleObstacles():
    def __init__(self):
        self.obstacles_list = []
    def getObjectsByColors(self, objectColor):
        return [ob for ob in self.obstacles_list if ob.color == objectColor]

    def getBall(self):
        Balls = self.getObjectsByColors('yellow')
        if len(Balls) != 0:
            cen = Balls[0].x+Balls[0].w/2
            return cen
        else:
            return None
    def sort(self):
        self.obstacles_list = sorted(self.obstacles_list,key=operator.attrgetter('x'))

    def findGaps(self):
        self.sort()
        prevEnd = int()
        maxGap = self.obstacles_list[0].x
        centreOfGap = (self.obstacles_list[0].x )/2
        self.obstacles_list.append(GetSingleObstacle(640,0,0,0,"blue"))
        for obstacle in self.obstacles_list:
            if prevEnd != 0:
                gap = obstacle.x - prevEnd
                if gap > maxGap:
                    maxGap = gap
                    centreOfGap = gap/2 + prevEnd
                    #print centreOfGap
            prevEnd = obstacle.end
        print centreOfGap


def findGap():
    global MultipleObstacles
    if len(MultipleObstacles.obstacles_list) >0:
        MultipleObstacles.findGaps()

def detectBall():
    global MultipleObstacles
    range = 50
    Ball = MultipleObstacles.getBall()
    if Ball != None:
        if Ball > 320 + range:
            motion =  'right'
        elif Ball < 320 - range:
            motion = 'left'
        else:
            motion =  'walk'
    else:
        motion = 'stop'
    rospy.loginfo(motion)
    motionPublisher.publish(motion)

def appendObstacles(msg):
    global prevFrame, MultipleObstacles, BufferObstacles
    SingleObstacle = GetSingleObstacle(msg.x, msg.y, msg.w, msg.h, msg.color)
    if msg.frame != prevFrame:
        if BufferObstacles != None:
            MultipleObstacles = BufferObstacles
            BufferObstacles = GetMultipleObstacles()
    BufferObstacles.obstacles_list.append(SingleObstacle)
    prevFrame = msg.frame


if __name__ == '__main__':
    prevFrame = 0
    MultipleObstacles = GetMultipleObstacles()
    BufferObstacles = GetMultipleObstacles()
    rospy.init_node('Intermediate', anonymous=True)
    rospy.Subscriber("obstacles", Obstacle, appendObstacles)
    motionPublisher = rospy.Publisher('motion', String, queue_size=10)
    while True:
        findGap()
    rospy.spin()
