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

class BlueObstacle():
    def __init__(self):
        self.found = False
        self.ledge = None
        self.redge = None

    def update(self,obstacle):
        if obstacle==None:
            self.found = False
            self.ledge = None
            self.redge = None
        else:
            self.found = True
            self.ledge = obstacle[0]
            self.redge = obstacle[1]

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

    def getBlueObstacle(self):
        obstacles = self.getObjectsByColors('blue')
        if len(obstacles) != 0:
            ledge = obstacles[0]
            redge = obstacles[0].x+obstacles[0].w
            return ledge,redge
        else:
            return None

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



def allignHead():
    if ball.x < 210:
        try:
            deg = 1 - (ball.x / 212.0)
            moco.publish('hl' + str(deg*0.8))
        except:
            pass
    elif ball.x > 420:
        try:
            deg = (ball.x - 418) / 222.0
            moco.publish('hr' + str(deg*0.8))
        except:
            pass
    elif ball.y < 160:
        try:
            deg = 1 - (ball.y / 162.0)
            moco.publish('hu' + str(deg*0.8))
        except:
            pass
    elif ball.y > 320:
        try:
            deg = (ball.y - 318) / 162.0
            moco.publish('hd' + str(deg*0.8))
        except:
            pass
    else:
        return "center"
    return ""


def ifBallFound():
    if ball.found:
        return 'fo'
    else:
        return 'nofo'

def ifBlueObstacleFound():
    if blueObstacle.found:
        return 'fo'
    else:
        return 'nofo'

def ifOtherObstacleFound():
    if otherObstacle.found:
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
    blueObstacle.update(MultipleObstacles.getBlueObstacle())
    redObstacle.update(MultipleObstacles.getRedObstacle())
    yellowObstacle.update(MultipleObstacles.getYellowObstacle())
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
blueObstacle = BlueObstacle()
yellowObstacle = YellowObstacle()
redObstacle = RedObstacle()
pan_angle = 0
tilt_angle = 90
command = 'hd45'
ball_buffer = []
#
# root  = State(state = 'Start',transdict={'fo':'exit','nofo':'L1'},val={'tilt':90,'pan':0})
# scan_fsm = FSM(root)
# scan_fsm.add(State(state='L1', transdict={'fo': 'exit', 'nofo': 'CU'}, val={'tilt': 90, 'pan': 60}))
# scan_fsm.add(State(state='CU', transdict={'fo': 'exit', 'nofo': 'R1'}, val={'tilt': 90, 'pan': 0}))
# scan_fsm.add(State(state='R1', transdict={'fo': 'exit', 'nofo': 'R2'}, val={'tilt': 90, 'pan': -60}))
# scan_fsm.add(State(state='R2', transdict={'fo': 'exit', 'nofo': 'CD'}, val={'tilt': 45, 'pan': -60}))
# scan_fsm.add(State(state='CD', transdict={'fo': 'exit', 'nofo': 'L2'}, val={'tilt': 45, 'pan': 0}))
# scan_fsm.add(State(state='L2', transdict={'fo': 'exit', 'nofo': 'Start'}, val={'tilt': 45, 'pan': 60}))
# scan_fsm.add(State(state='exit',transdict={'fo': 'exit', 'nofo': 'Start'}, val={'tilt': 90, 'pan': 0}))


def gen_msg(ta,pa):
    return "ssT" + str(ta)+"P"+str(pa)
#
# def simple_scan():
#     moco.publish(gen_msg(75, 0))
#     time.sleep(1)
#     if ball.found:
#         return 'center'

def sweep():
    global ball_buffer
    for tilt in range(80,5,-25):
        for pan in range(60,-120,-60):
            ball_buffer = []
            moco.publish(gen_msg(tilt, pan))
            time.sleep(1.5)
            if 'fo' in ball_buffer:
                finalAllign()
                return 'found'


def finalAllign():
    print "Alligning..."
    start_time = time.time()
    diff = 0
    while allignHead()!= "center":
        if not ball.found:
            diff = time.time() - start_time
        else:
            diff = 0
            start_time = time.time()
        if diff > 2.0:
            return "fail"
        # if not ball.found:
        #     break
    print "Alligned"
    return

def find():
    while not ball.found:
         if sweep() == 'found':
             break
    if pan_angle > 0:
        step = "ls"
    else:
        step = "rs"
    while not float_check(pan_angle,0):
        print "Chala"
        k = finalAllign()
        if k == "fail":
            return
        time.sleep(1)
        moco.publish(step)
        print ball.x
        time.sleep(1)
    moco.publish("ba")





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
    isYellowObstacle = False
    escapedBlue = False
    MultipleObstacles = GetMultipleObstacles()
    BufferObstacles = GetMultipleObstacles()
    rospy.init_node('Intermediate', anonymous=True)
    rospy.Subscriber("obstacles", Obstacle, appendObstacles)
    rospy.Subscriber("feedback", String, feedback_handler)
    moco = rospy.Publisher('moco', String, queue_size=1)
    time.sleep(0.5)
    # raw_input("Kick>")


    #check =  MultipleObstacles.getObstacle()
    # if ball.found:
    #     if check == False:
    #         moco.publish("sk")
    #     else:
    #         moco.publish("rk")

    moco.publish(gen_msg(60, 0))
    time.sleep(0.5)

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
        moco.publish(gen_msg(80, 0))
        while redObstacle.found:
            direction="rs"
            moco.publish(direction)
            moco.publish(direction)
            moco.publish(direction)
            time.sleep(.5)
        moco.publish(gen_msg(60, 0))
        while blueObstacle.found or yellowObstacle.found:
            if yellowObstacle.found:
                isYellowObstacle = True
            if isYellowObstacle:
                moco.publish("rs")
            else:
                moco.publish("ls")
            time.sleep(0.5)
            escapedBlue = True
        if escapedBlue:
            moco.publish(gen_msg(80, 0))
            while redObstacle.found:
                direction="ls"
                moco.publish(direction)
                moco.publish(direction)
                moco.publish(direction)
                time.sleep(.5)
                escapedRed = True
            moco.publish(gen_msg(60, 0))
        if escapedRed:
            continue
        moco.publish("sw")
        time.sleep(1)











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
