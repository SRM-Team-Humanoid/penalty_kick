#!/usr/bin/env python
import rospy
from penalty_kick.msg import Obstacle
from std_msgs.msg import String
from ImagePublisher import GetSingleObstacle
import time
import readchar
import operator


#-----Params-----------------

tilt_thresh = 90
up_scan = 0
down_scan = 50

#-----------------------------

class GetMultipleObstacles():
    def __init__(self,offset):
        self.obstacles_list = []
        self.offset = offset
        self.side_offset = 120
    def getObjectsByColors(self, objectColor):
        return [ob for ob in self.obstacles_list if ob.color == objectColor]

    def getCenterObstacle(self):
        for ob in self.obstacles_list:
             print "ob " + ob.color+ " "+str(ob.w)+" "+str(ob.x)+" "+str((ob.x+ob.w)/2)
             if ob.y + ob.h > 150 :
                 print "ob-close "+ str(ob.x + ob.h)
                 return "ob-close"
             elif ob.x < 640-self.offset and  ob.x+ob.w >self.offset:
                 print "ob "+ str(ob.x)+ " "+ob.color
                 return "center"
             elif ob.x < self.side_offset and ob.w>0:
                 print "left ob " + str(ob.x + ob.w) + " " + ob.color
                 return "left"
             elif ob.x > 640-self.side_offset:
                 print "right ob " + str(ob.x) + " " + ob.color
                 return "right"
        return "no-ob"

    def getRedObstacle(self):
            for ob in self.obstacles_list:
                print "ob "+ ob.color
                if ob.x < 640 - self.offset and ob.x + ob.w > self.offset and ob.color == 'red':
                    print "ob " + str(ob.x) + str(ob.x+ob.w)
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
            BufferObstacles = GetMultipleObstacles(offset = 120)
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
    print "checking red..."
    time.sleep(delay)
    #raw_input("red check?")
    return MultipleObstacles.getRedObstacle()

def sideWalk(steps):
    global broken
    i =0
    if not broken:
        print "left"
        while not float_check(pan_angle,head_left):
            moco.publish(gen_msg(head_down, head_left))
        time.sleep(delay)
        #raw_input("start side walk")
        print "starting side walk..."
        for i in range(1,steps + 1):
                if MultipleObstacles.getCenterObstacle()=="ob-close":
                    broken = True
                    break
                else:
                    moco.publish('ls')
                    time.sleep(0.5)
    if broken:
        print "right"
        while not float_check(pan_angle, head_right):
            moco.publish(gen_msg(head_down, head_right))
        time.sleep(delay)
        #raw_input("start side walk2")
        print "starting side walk2..."
        for j in range(1, i+steps + 1):
            if MultipleObstacles.getCenterObstacle() == "ob-close":
                moco.publish("sw")
                broken = False
                break
            else:
                moco.publish('rs')
                time.sleep(0.5)

broken = False

if __name__ == '__main__':
    prevFrame = 0
    MultipleObstacles = GetMultipleObstacles(offset = 120)
    BufferObstacles = GetMultipleObstacles(offset = 120)
    rospy.init_node('Intermediate', anonymous=True)
    rospy.Subscriber("obstacles", Obstacle, appendObstacles)
    rospy.Subscriber("feedback", String, feedback_handler)
    moco = rospy.Publisher('moco', String, queue_size=1)
    head_up = 90
    head_down = 20
    head_left = 80
    head_right = -80
    delay = 1
    isRedObstacle = checkRed()
    moco.publish(gen_msg(head_down, 0))
    time.sleep(delay)
    print isRedObstacle
    raw_input()
    while True:
      if isRedObstacle:
         print "red obstacle detected on path"
         if not checkRed():
             sideWalk(steps=2)
             isRedObstacle = False
         moco.publish(gen_msg(head_down, 0))
         time.sleep(delay)
         #raw_input()
      if MultipleObstacles.getCenterObstacle() == "center":
        sideWalk(steps=2)
        time.sleep(delay)
        isRedObstacle = checkRed()
        moco.publish(gen_msg(head_down, 0))
        time.sleep(delay)
      elif MultipleObstacles.getCenterObstacle() == "left":
          print "left ob -- moving right"
          moco.publish('rs')
      elif MultipleObstacles.getCenterObstacle() == "right":
          print "right ob -- moving left"
          moco.publish('ls')
      else:
        moco.publish("sw")
        broken = False
      print isRedObstacle
      #raw_input()
      print "TAKING NEXT STEP"
      time.sleep(delay)