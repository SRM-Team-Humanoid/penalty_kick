#!/usr/bin/env python
import rospy
from penalty_kick.msg import Obstacle
from std_msgs.msg import String
from ImagePublisher import GetSingleObstacle
import time
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

class State(object):
    def __init__(self,state,transdict={},val={}):
        self.state = state
        self.trans = transdict
        self.val = val

    def __str__(self):
        return str(self.state)

    def transit(self,trigger):
        try:
            return self.trans[trigger]
        except:
            raise KeyError("Unknown Trigger")

class FSM(object):

    def __init__(self,state):
        self.current = state
        self.prev = None
        self.states = [state]

    def add(self,state):
        self.states.append(state)

    def Find(self,state):
        for s in self.states:
            if state == str(s):
                return s

        raise KeyError("Unknown State")

    def update(self,trigger):
        temp = self.current
        self.current = self.Find(self.current.transit(trigger))
        self.prev = temp


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


    def getBall(self):
        Balls = self.getObjectsByColors('yellow')
        if len(Balls) != 0:
            xcen = Balls[0].x+Balls[0].w/2
            ycen = Balls[0].y+Balls[0].h/2
            return xcen,ycen
        else:
            return None





def allignHead():
    if ball.x < 210:
        try:
            deg = 1 - (ball.x / 212.0)
            moco.publish('hl' + str(deg))
        except:
            pass
    elif ball.x > 420:
        try:
            deg = (ball.x - 418) / 222.0
            moco.publish('hr' + str(deg))
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


def ifBallFound():
    if ball.found:
        return 'fo'
    else:
        return 'nofo'

def allignX():
    return ball.x in range(210, 420)

def appendObstacles(msg):
    global prevFrame, MultipleObstacles, BufferObstacles,ball,ball_buffer
    SingleObstacle = GetSingleObstacle(msg.x, msg.y, msg.w, msg.h, msg.color)
    if msg.frame != prevFrame:
        if BufferObstacles != None:
            MultipleObstacles = BufferObstacles
            BufferObstacles = GetMultipleObstacles()
    BufferObstacles.obstacles_list.append(SingleObstacle)
    ball.update(MultipleObstacles.getBall())
    ball_buffer.append(ifBallFound())
    prevFrame = msg.frame

def feedback_handler(data):
    global pan_angle,tilt_angle
    feedback = data.data
    feedback = map(float,feedback.split())
    pan_angle = feedback[0]
    tilt_angle = feedback[1]


def getBallStatus():
    fo = sum([1 for x in ball_buffer if x =='fo'])
    nofo = sum([1 for x in ball_buffer if x == 'nofo'])
    print fo,nofo

    if fo > nofo:
        return 'fo'
    else:
        return 'nofo'



ball = Ball()
pan_angle = 0
tilt_angle = 90
command = 'hd45'
ball_buffer = []

root  = State(state = 'Start',transdict={'fo':'exit','nofo':'L1'},val={'tilt':90,'pan':0})
scan_fsm = FSM(root)
scan_fsm.add(State(state='L1', transdict={'fo': 'exit', 'nofo': 'CU'}, val={'tilt': 90, 'pan': 60}))
scan_fsm.add(State(state='CU', transdict={'fo': 'exit', 'nofo': 'R1'}, val={'tilt': 90, 'pan': 0}))
scan_fsm.add(State(state='R1', transdict={'fo': 'exit', 'nofo': 'R2'}, val={'tilt': 90, 'pan': -60}))
scan_fsm.add(State(state='R2', transdict={'fo': 'exit', 'nofo': 'CD'}, val={'tilt': 45, 'pan': -60}))
scan_fsm.add(State(state='CD', transdict={'fo': 'exit', 'nofo': 'L2'}, val={'tilt': 45, 'pan': 0}))
scan_fsm.add(State(state='L2', transdict={'fo': 'exit', 'nofo': 'Start'}, val={'tilt': 45, 'pan': 60}))
scan_fsm.add(State(state='exit',transdict={'fo': 'exit', 'nofo': 'Start'}, val={'tilt': 90, 'pan': 0}))


def gen_msg(ta,pa):
    return "ssT" + str(ta)+"P"+str(pa)
#
# def simple_scan():
#     moco.publish(gen_msg(75, 0))
#     time.sleep(1)
#     if ball.found:
#         return 'center'

def sweep():
    for tilt in range(100,40,-20):
        for pan in range(60,-60,-30):
            moco.publish(gen_msg(tilt, pan))
            time.sleep(1)
            if ball.found:
                allignHead()
                return


def finalAllign():
    while not ball.center:
        print "Alligning..."
        allignHead()
        # if not ball.found:
        #     break
    return




# while True:
#     print scan_fsm.current,scan_fsm.current.val
#     trig = raw_input()
#     scan_fsm.update(trig)

def float_check(a,b):
    if abs(a-b)<10:
        return True
    return False

if __name__ == '__main__':
    prevFrame = 0
    MultipleObstacles = GetMultipleObstacles()
    BufferObstacles = GetMultipleObstacles()
    rospy.init_node('Intermediate', anonymous=True)
    rospy.Subscriber("obstacles", Obstacle, appendObstacles)
    rospy.Subscriber("feedback", String, feedback_handler)
    moco = rospy.Publisher('moco', String, queue_size=1)
    time.sleep(0.5)
    raw_input("Kick>")
    #check =  MultipleObstacles.getObstacle()


    # if ball.found:
    #     if check == False:
    #         moco.publish("sk")
    #     else:
    #         moco.publish("rk")

    moco.publish(gen_msg(90, 0))
    moco.publish("rk")
    time.sleep(1)
    moco.publish("ba")
    time.sleep(0.5)
    raw_input("Begin")
    while not ball.found:
        sweep()
    if pan_angle > 0:
        step = "ls"
    else:
        step = "rs"

    while not float_check(pan_angle,0):
        print "Chala"
        moco.publish(step)
        print ball.x
        time.sleep(1)
        finalAllign()
        time.sleep(1)
    moco.publish("ba")

    raw_input()
    while tilt_angle>50:
        moco.publish("sw")
        # while not allignX():
        #     if ball.x < 210:
        #         moco.publish("ls")
        #         time.sleep(0.4)
        #     else:
        #         moco.publish("rs")
        #         time.sleep(0.4)
        # finalAllign()
        time.sleep(0.4)
    raw_input()
    moco.publish("sk")











    # while True:
    #     allignHead()
    #     time.sleep(0.5)
    #     moco.publish()
    #
    # while True:
    #     status = getBallStatus()
    #     # if str(scan_fsm.current) == 'exit':
    #     #     scan_fsm.update(status)
    #     # if not (float_check(pan_angle,scan_fsm.current.val['pan']) and float_check(tilt_angle,scan_fsm.current.val['tilt'])):
    #     #     continue
    #     # print status
    #     scan_fsm.update(status)
    #     if str(scan_fsm.current)!='exit':
    #         msg = gen_msg(scan_fsm.current.val['tilt'], scan_fsm.current.val['pan'])
    #         moco.publish(msg)
    #         ball_buffer = []
    #         time.sleep(1.5)
    #     print scan_fsm.current
    #
    # moco.publish('none')
    #
    rospy.spin()
