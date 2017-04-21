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

class BlueObstacle():
    def __init__(self):
        self.found = False
        self.center = False
        self.ledge = None
        self.redge = None

    def update(self,obstacles):
        if len(obstacles) == 0:
            self.found = False
            self.ledge = None
            self.redge = None
            self.center = False
        else:
            self.found = True
            for obstacle in obstacles:
                if obstacle.x in range(210,420) or obstacle.x+obstacle.w in range(210,420):
                    self.center = True
                    self.ledge = obstacle.x
                    self.redge = obstacle.x + obstacle.w

class RedObstacle():
    def __init__(self):
        self.found = False

    def update(self,obstacle):
        if obstacle==False:
            self.found = False
        else:
            self.found = True

class YellowObstacle():
    def __init__(self):
        self.found = False

    def update(self,obstacle):
        if obstacle==False:
            self.found = False
        else:
            self.found = True

class GetMultipleObstacles():
    def __init__(self):
        self.obstacles_list = []
    def getObjectsByColors(self, objectColor):
        return [ob for ob in self.obstacles_list if ob.color == objectColor]

    def getObstacle(self):
        for ob in self.obstacles_list:
            if ob.x+ob.w/2 in range(280,360):
                print "ob "+ str(ob.x)
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

    def getRedObstacle(self):
        obstacles = self.getObjectsByColors('red')
        if len(obstacles) == 0:
            return False
        else:
            return True

    def getYellowObstacle(self):
        obstacles = self.getObjectsByColors('yellow')
        if len(obstacles) == 0:
            return False
        else:
            return True

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
    blueObstacle.update(MultipleObstacles.getObjectsByColors('blue'))
    redObstacle.update(MultipleObstacles.getRedObstacle())
    yellowObstacle.update(MultipleObstacles.getYellowObstacle())
    prevFrame = msg.frame

def feedback_handler(data):
    global pan_angle,tilt_angle
    feedback = data.data
    feedback = map(float,feedback.split())
    pan_angle = feedback[0]
    tilt_angle = feedback[1]

blueObstacle = BlueObstacle()
yellowObstacle = YellowObstacle()
redObstacle = RedObstacle()
pan_angle = 0
tilt_angle = 90
command = 'hd45'


def gen_msg(ta,pa):
    return "ssT" + str(ta)+"P"+str(pa)

def float_check(a,b):
    if abs(a-b)<10:
        return True
    return False

if __name__ == '__main__':
    prevFrame = 0
    isYellowObstacle = False
    escapedBlue = False
    MultipleObstacles = GetMultipleObstacles()
    BufferObstacles = GetMultipleObstacles()
    rospy.init_node('Intermediate', anonymous=True)
    rospy.Subscriber("obstacles", Obstacle, appendObstacles)
    rospy.Subscriber("feedback", String, feedback_handler)
    moco = rospy.Publisher('moco', String, queue_size=1)
    time.sleep(0.5)
    moco.publish(gen_msg(90, 0))
    time.sleep(0.5)
    head_up = 60
    head_down = 40
    raw_input()
    while True:
        # while not blueObstacle.found:
        #     moco.publish(gen_msg(80, 0))
        #     while redObstacle.found:
        #         moco.publish("r+s")
        #         moco.publish("rs")
        #         moco.publish("rs")
        #         time.sleep(.5)
        #     moco.publish(gen_msg(60, 0))
        #     if not blueObstacle.found:
        #         moco.publish("sw")
        #     time.sleep(1)

        escapedBlue = False
        escapedRed = False
        isYellowObstacle = False
        moco.publish(gen_msg(head_up, 0))
        while redObstacle.found:
            direction="rs"
            moco.publish(direction)
            moco.publish(direction)
            moco.publish(direction)
            time.sleep(.5)
        moco.publish(gen_msg(head_down, 0))
        time.sleep(0.5)
        while blueObstacle.center or yellowObstacle.found:
            if yellowObstacle.found:
                isYellowObstacle = True
            if isYellowObstacle:
                moco.publish("rs")
            else:
                moco.publish("ls")
            print isYellowObstacle
            time.sleep(0.5)
            escapedBlue = True
        if escapedBlue:
            moco.publish(gen_msg(head_up, 0))
            time.sleep(0.5)
            while redObsstacle.found:
                direction="ls"
                moco.publish(direction)
                moco.publish(direction)
                moco.publish(direction)
                time.sleep(.5)
                escapedRed = True
            moco.publish(gen_msg(head_down, 0))
            time.sleep(0.5)
        if escapedRed:
            continue
        moco.publish("sw")
        time.sleep(0.5)
    rospy.spin()

