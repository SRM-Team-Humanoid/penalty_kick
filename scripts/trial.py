#!/usr/bin/env python
import rospy
from penalty_kick.msg import Obstacle
from std_msgs.msg import String
from ImagePublisher import GetSingleObstacle
import time
from readchar import readchar
import operator


#-----Params-----------------

tilt_thresh = 90
up_scan = 0
down_scan = 50

#-----------------------------

class GetMultipleObstacles():
    def __init__(self):
        self.obstacles_list = []
    def getObjectsByColors(self, objectColor):
        return [ob for ob in self.obstacles_list if ob.color == objectColor]

    def getCenterObstacle(self):
        for ob in self.obstacles_list:
            print "ob " + ob.color+ " "+str(ob.w)+" "+str(ob.x)+" "+str((ob.x+ob.w)/2)
            if ob.x <400 and  ob.x+ob.w >240:
                print "ob "+ str(ob.x)+ " "+ob.color
                return True
        return False

    def getRedObstacle(self):
            for ob in self.obstacles_list:
                print "ob "+ ob.color
                if  ob.x <400 and  ob.x+ob.w >240 and ob.color == 'red':
                    print "ob " + str(ob.x)
                    return True
            return False


            # def getBlueObstacle(self):
    #     obstacles = self.getObjectsByColors('blue')
    #     if len(obstacles) != 0:
    #         ledge = obstacles[0]
    #         redge = obstacles[0].x+obstacles[0].w
    #         return ledge,redge
    #     else:
    #         return None



 # def allignX():
 #     return blueObstacle.x + blueObstacle.w/2 in range(210, 420)

def appendObstacles(msg):
    global prevFrame, MultipleObstacles, BufferObstacles
    SingleObstacle = GetSingleObstacle(msg.x, msg.y, msg.w, msg.h, msg.color)
    if msg.frame != prevFrame:
        if BufferObstacles != None:
            MultipleObstacles = BufferObstacles
            BufferObstacles = GetMultipleObstacles()
    BufferObstacles.obstacles_list.append(SingleObstacle)
    prevFrame = msg.frame

def feedback_handler(data):
    global pan_angle,tilt_angle
    feedback = data.data
    feedback = map(float,feedback.split())
    pan_angle = feedback[0]
    tilt_angle = feedback[1]

pan_angle = 0
tilt_angle = 90
command = 'hd45'


def gen_msg(ta,pa):
    return "ssT" + str(ta)+"P"+str(pa)

def float_check(a,b):
    if abs(a-b)<10:
        return True
    return False


def checkRed():
    moco.publish(gen_msg(head_up, 0))
    time.sleep(delay)
    return MultipleObstacles.getRedObstacle()

def sideWalk(steps):
    #TODO remove all comments after back walk
    #broken = True
    #while broken:
        broken = False
        print "left"
        moco.publish(gen_msg(head_down, head_left))
        time.sleep(delay)
        for i in range(1,  steps + 1):
            if MultipleObstacles.getCenterObstacle():
                broken = True
                break
            else:
                moco.publish('ls')
        if broken:
            print "right"
            moco.publish(gen_msg(head_down, head_right))
            time.sleep(delay)
            for j in range(1, i+steps + 1):
                if MultipleObstacles.getCenterObstacle():
                    broken = True
                    break
                else:
                    moco.publish('rs')
            if j == i+steps:
                broken = False
        if broken:
                # to do back walk
                pass


if __name__ == '__main__':
    prevFrame = 0
    MultipleObstacles = GetMultipleObstacles()
    BufferObstacles = GetMultipleObstacles()
    rospy.init_node('Intermediate', anonymous=True)
    rospy.Subscriber("obstacles", Obstacle, appendObstacles)
    rospy.Subscriber("feedback", String, feedback_handler)
    moco = rospy.Publisher('moco', String, queue_size=1)
    head_up = 90
    head_down = 35
    head_left = 80
    head_right = -80
    delay =1
    raw_input()
    isRedObstacle = checkRed()
    moco.publish(gen_msg(head_down, 0))
    time.sleep(delay)
    print isRedObstacle
    raw_input()
    while True:
      if isRedObstacle:
         if not checkRed():
             sideWalk(steps=3)
             isRedObstacle = False
         moco.publish(gen_msg(head_down, 0))
         time.sleep(delay)
      if MultipleObstacles.getCenterObstacle():
        sideWalk(steps=2)
        isRedObstacle = checkRed()
        moco.publish(gen_msg(head_down, 0))
        time.sleep(delay)
      else:
        moco.publish("sw")
      print isRedObstacle
      raw_input()
    rospy.spin()
