#!/usr/bin/env python
import rospy
from penalty_kick.msg import Obstacle
from std_msgs.msg import String
from ImagePublisher import GetSingleObstacle
import operator




#-----Params-----------------


# path = "/home/warr/cdatkin_ws/src/penalty_kick/scripts/"
# fo = open(path+"foo.txt", "w")
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






def allignHead():
    global pan_angle,tilt_angle
    if ball.x < 210:
        try:
            deg = 1 - (ball.x / 212.0)
            moco.publish('hr' + str(deg))
        except:
            pass
    elif ball.x > 420:
        try:
            deg = (ball.x - 418) / 222.0
            moco.publish('hl' + str(deg))
        except:
            pass
    elif ball.y < 160:
        try:
            deg = 1 - (ball.y / 162.0)
            moco.publish('hu' + str(deg))
        except:
            pass
    elif ball.y > 320:
        try:
            deg = (ball.y - 318) / 162.0
            moco.publish('hd' + str(deg))
        except:
            pass



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

def feedback_handler(data):
    global pan_angle,tilt_angle
    feedback = data.data
    feedback = map(float,feedback.split())
    # print feedback
    pan_angle = feedback[0]
    tilt_angle = feedback[1]




ball = Ball()
pan_angle = 0
tilt_angle = 90
command = 'hd45'

# command_list = ['hl60','hr60','hr60','hd45','hl60','hl60','hr60','hu45''bs']

if __name__ == '__main__':
    prevFrame = 0
    MultipleObstacles = GetMultipleObstacles()
    BufferObstacles = GetMultipleObstacles()
    rospy.init_node('Intermediate', anonymous=True)
    rospy.Subscriber("obstacles", Obstacle, appendObstacles)
    rospy.Subscriber("feedback", String, feedback_handler)
    moco = rospy.Publisher('moco', String, queue_size=1)
    leftpan_index = 0
    isDown=False
    rightpan_index = 0
    isRightmost = False
    def pan_left_to_right():
        command_arrray=['hl60','hr60','hr60']
        moco.publish(command_arrray[rightpan_index])
        if rightpan_index==2:
            global isRightmost = True
            rightpan_index=0
        global rightpan_index += 1
    def pan_right_to_left():
        command_arrray=['hl60','hl60']
        moco.publish(command_arrray[leftpan_index])
        if leftpan_index==1:
            global isRightmost = False
            leftpan_index=0
        global leftpan_index += 1
    while True:
        # while not ball.found:
        while not ball.found:
            if (not isRightmost):
                if isDown==True:
                    moco.publish('hu45')
                    moco.publish('hr60')
                pan_left_to_right()
            else:
                moco.publish('hd45')
                isDown=True
                if ball.found:
                    break
                pan_right_to_left()
            if ball.found:
                break
            raw_input(">")
            # for c in command_list:
            #     raw_input(">")
            #     moco.publish(c)
            #     # if ball.found:
                #     break
        # while not ball.center and ball.x!=None and ball.y!=None:
        #     allignHead()
        #     print tilt_angle,pan_angle


        moco.publish('none')

    rospy.spin()


