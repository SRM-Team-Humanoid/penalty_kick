#!/usr/bin/env python
import random
import pypot.dynamixel
import time
import numpy as np
from pprint import pprint
import xml.etree.cElementTree as ET
from collections import Counter
from copy import deepcopy
import rospy
from std_msgs.msg import String

path = "/home/warr/cdatkin_ws/src/penalty_kick/scripts/"

class Dxl(object):
    def __init__(self,port_id=0, scan_limit=25, lock=-1,debug=False):
        # Initializes Dynamixel Object
        # Port ID is zero by default
        ports = pypot.dynamixel.get_available_ports()
        if not ports:
            raise IOError('no port found!')

        print('ports found', ports)
        print('connecting on the first available port:', ports[port_id])
        dxl_io = pypot.dynamixel.DxlIO(ports[port_id])
        ids = dxl_io.scan(range(25))
        print(ids)

        if lock > 0:
            if len(ids) < lock:
                raise RuntimeError("Couldn't detect all motors.")

        self.dxl_io = dxl_io
        self.ids = ids
        debug_speeds = [30 for x in ids]
        unleash = [1023 for x in ids]
        if debug:
            dxl_io.set_moving_speed(dict(zip(ids,debug_speeds)))
        else:
            dxl_io.set_moving_speed(dict(zip(ids, unleash)))

    def directWrite(self,dicta):
        self.dxl_io.set_goal_position(dicta)

    def setPos(self,pose):
        '''
        for k in pose.keys():
            if k not in self.ids:
                del pose[k]
        '''
        writ = {key: value for key, value in pose.items() if key in self.ids}
        #print writ
        self.dxl_io.set_goal_position(writ)

    def getPos(self):
        return Motion(1," ".join(map(str,self.dxl_io.get_present_position(self.ids))),0)

    def publish(self,pub):
        pub.publish(" ".join(map(str,self.dxl_io.get_present_position([19,20]))))



class XmlTree(object):

    def __init__(self,str):
        try:
            with open(str) as f:
                pass
            self.tree = ET.ElementTree(file=str)
        except:
            raise RuntimeError("File not found.")

    def parsexml(self,text):
        find = "PageRoot/Page[@name='" + text + "']/steps/step"
        motions = []
        prev_frame = 0
        steps = [x for x in self.tree.findall(find)]
        if len(steps)==0:
            # print find
            raise RuntimeError("ParseFail!")
        for step in steps:
            motion = Motion(step.attrib['frame'], step.attrib['pose'], prev_frame)
            prev_frame = step.attrib['frame']
            motions.append(motion)

        return motions

    def superparsexml(self,text,exclude=[],offsets=[]):
        find = "FlowRoot/Flow[@name='"+text+"']/units/unit"
        steps = [x for x in self.tree.findall(find)]
        if len(steps)==0:
            # print find
            raise RuntimeError("ParseFail!")
        motionsets = []
        for step in steps:
            motionsets.append(MotionSet(self.parsexml(step.attrib['main']),speed=float(step.attrib['mainSpeed']),exclude=exclude,offsets=offsets))

        return motionsets



class Motion(object):
    def __init__(self,frame,pose,prev_frame):
        self.frame = int(frame)
        self.pose = {}
        self.delay = self.frame-int(prev_frame)
        for i,p in enumerate(pose.split()):
            self.pose[i+1] =float(p)

    def __str__(self):
        return "Frame:"+str(self.frame) + "      Delay:"+str(self.delay) + "     Pose:"+" ".join(map(str,self.pose.values()))

    def updatePose(self,offset,add=True):
        if add:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] += offset[k]
        else:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] -= offset[k]


    def write(self,state, speed,exclude=[],offset={}):
        begpos = state.pose
        endpos = self.pose
        frames = []
        ids = []
        for k in endpos.keys():
            try:
                begpos[k]
            except:
                begpos[k]=0
            if begpos[k]!=endpos[k] and k not in exclude:
                frames.append(np.linspace(begpos[k],endpos[k],self.delay))
                ids.append(k)

        frames = zip(*frames)
        for f in frames:
            writ = dict(zip(ids, f))
            dxl.setPos(writ)
            time.sleep(0.008 / speed)
            # print writ



class MotionSet(object):
    def __init__(self,motions,speed=1.0,exclude =[],offsets=[]):
        self.motions = motions
        self.speed = speed
        self.exclude = exclude
        self.offsets = offsets
        self.loaded = False

    def setExclude(self,list):
        self.exclude = list

    def setSpeed(self,speed):
        self.speed = speed

    def stateUpdater(self,motion):
        global state
        for k in motion.pose.keys():
            state.pose[k]=motion.pose[k]

    def execute(self,speed=-1,iter=1):
        if speed<0:
            speed = self.speed
        if not self.loaded:
            for offset in self.offsets:
                for motion in self.motions:
                    motion.updatePose(offset)
            self.loaded = True

        while iter>0:
            for motion in self.motions:
                motion.write(state,speed,self.exclude)
                self.stateUpdater(motion)
            iter-=1

class Action():
    def __init__(self,motionsets):
        self.motionsets=motionsets

    def add(self,motionsets):
        self.motionsets.extend(motionsets)

    def execute(self,iter=1,speed=1):
        while iter>0:
            for motionset in self.motionsets:
                # for m in motionset.motions:
                #     print m
                orig = motionset.speed
                motionset.speed = motionset.speed*speed
                motionset.execute()
                motionset.speed = orig
            iter -= 1

class Head():
    def __init__(self,dxl):
        self.dxl = dxl
        self.pan_motor  = 19
        self.tilt_motor = 20
        self.pan_angle = 0
        self.tilt_angle = 90
        self.write()

    def updateState(self,dicta):
        global state
        state.updatePose(dicta)

    def write(self):
        self.dxl.directWrite({self.pan_motor:self.pan_angle,self.tilt_motor:self.tilt_angle})



    def pan_left(self,deg=1.0):
        self.pan_angle += deg
        self.updateState({self.pan_motor:deg})
        self.write()
        fac = abs(deg)
        time.sleep(0.008*fac)

    def pan_right(self,deg=1.0):
        self.pan_angle -= deg
        self.updateState({self.pan_motor: -1*abs(deg)})
        self.write()
        fac = abs(deg)
        time.sleep(0.008 * fac)

    def tilt_up(self,deg=1.0):
        self.tilt_angle += deg
        self.updateState({self.tilt_motor: deg})
        self.write()
        fac = abs(deg)
        time.sleep(0.008*fac)

    def tilt_down(self,deg=1.0):
        self.tilt_angle -= deg
        self.updateState({self.tilt_motor: -1*abs(deg)})
        self.write()
        fac = abs(deg)
        time.sleep(0.008 * fac)

    def set(self,tilt,pan):
        t_deg = tilt-self.tilt_angle
        p_deg = pan-self.pan_angle
        self.tilt_angle = tilt
        self.pan_angle = pan
        self.updateState({self.tilt_motor:t_deg,self.pan_motor:p_deg})
        self.write()
        fac = max(abs(t_deg),abs(p_deg))
        time.sleep(0.008 * fac)



def moco(data):
    cmdDeg = data.data
    print cmdDeg
    cmd = cmdDeg[0:2]
    if cmd == 'hl':
        head.pan_left(float(cmdDeg[2:]))
    elif cmd == 'hr':
        head.pan_right(float(cmdDeg[2:]))
    elif cmd == 'hu':
        head.tilt_up(float(cmdDeg[2:]))
    elif cmd == 'hd':
        head.tilt_down(float(cmdDeg[2:]))
    elif cmd =='ss':
        t = cmdDeg.index('T')
        p = cmdDeg.index('P')
        tilt = float(cmdDeg[t+1:p])
        pan = float(cmdDeg[p+1:])
        head.set(tilt,pan)
    elif cmd == 'rk':
        rskick.execute(speed=0.5)
        time.sleep(0.2)
        r_turn.execute(iter=1)
    elif cmd == 'lk':
        lskick.execute()
    elif cmd == 'ls':
        left_side_step.execute()
    elif cmd == 'rs':
        right_side_step.execute()
    elif cmd =='ba':
        balance.execute()
    elif cmd == 'sk':
        kick.execute(speed=0.5)
    elif cmd == 'sw':
        balance.execute()
        time.sleep(0.5)
        boom_walk.execute(iter=4)
        time.sleep(0.5)
        balance.execute()

#----------------------------------------------------------------------------------------------------------------
darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
arm = {1:45, 2:-45}
hand = {5: 90, 6: -90}
hand_open = {5: -60, 6: 60}

tree = XmlTree(path+'data.xml')
tree2 = XmlTree(path+'soccer.xml')
tree3 = XmlTree(path+'fight.xml')
balance = MotionSet(tree.parsexml("152 Balance"), offsets=[darwin])
lskick = MotionSet(tree2.parsexml("40 Pass_L"), offsets=[darwin],speed = 1.5)
rskick = MotionSet(tree2.parsexml("39 Pass_R"), offsets=[darwin],speed = 1.5)
#kick = MotionSet(tree.parsexml("19 R kick"),speed=2,offsets=[darwin])
w1 = MotionSet(tree.parsexml("32 F_S_L"),speed=2.1,offsets=[darwin])
w2 = MotionSet(tree.parsexml("33 "),speed=2.1,offsets=[darwin])
w3 = MotionSet(tree.parsexml("38 F_M_R"),speed=2.7,offsets=[darwin])
w4 = MotionSet(tree.parsexml("39 "),speed=2.1,offsets=[darwin])
w5 = MotionSet(tree.parsexml("36 F_M_L"),speed=2.7,offsets=[darwin])
w6 = MotionSet(tree.parsexml("37 "),speed=2.1,offsets=[darwin])
r_turn = MotionSet(tree2.parsexml("27 RT"),speed=1.2,offsets=[darwin])
l_turn = MotionSet(tree2.parsexml("28 LT"),speed=1.2,offsets=[darwin])


lside1 = MotionSet(tree.parsexml("80 L_S_L"), offsets=[darwin,hand,arm],speed = 2.1)
lside2 = MotionSet(tree.parsexml("81 "), offsets=[darwin,hand,arm], speed = 2.1)
lside3 = MotionSet(tree.parsexml("86 L_M_R"), offsets=[darwin,hand,arm], speed = 2.7)
lside4 = MotionSet(tree.parsexml("87 "), offsets=[darwin,hand,arm], speed = 2.7)
lside5 = MotionSet(tree.parsexml("84 L_M_L"), offsets=[darwin,hand,arm], speed = 2.7)
lside6 = MotionSet(tree.parsexml("85 "), offsets=[darwin,hand,arm], speed = 2.7)

rside1 = MotionSet(tree.parsexml("92 R_S_L"), offsets=[darwin,hand,arm],speed = 2.1)
rside2 = MotionSet(tree.parsexml("93 "), offsets=[darwin,hand,arm], speed = 2.1)
rside3 = MotionSet(tree.parsexml("98 R_M_R"), offsets=[darwin,hand,arm], speed = 2.7)
rside4 = MotionSet(tree.parsexml("99 "), offsets=[darwin,hand,arm], speed = 2.7)
rside5 = MotionSet(tree.parsexml("96 R_M_L"), offsets=[darwin,hand,arm], speed = 2.7)
rside6 = MotionSet(tree.parsexml("97 "), offsets=[darwin,hand,arm], speed = 2.7)


# left_side_step = Action(tree2.superparsexml("19 L",offsets=[darwin]))
left_side_step = Action(tree2.superparsexml("21 Fst_L",offsets=[darwin]))
right_side_step = Action(tree2.superparsexml("20 Fst_R",offsets=[darwin]))

kick = Action(tree2.superparsexml("26 F_PShoot_R",offsets = [darwin]))

l_side_init = Action([lside1,lside2])
l_side_walk = Action([lside3,lside4,lside5,lside6])

r_side_init = Action([rside1,rside2])
r_side_walk = Action([rside3,rside4,rside5,rside6])

l_step = MotionSet(tree2.parsexml("10 ff_l_r"), speed=1.5, offsets=[darwin])
r_step = MotionSet(tree2.parsexml("9 ff_r_l"), speed=1.5, offsets=[darwin])
boom_walk = Action([l_step,r_step])
walk_init = Action([w1,w2])
walk_motion = Action([w3,w4,w5,w6])
#------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    dxl = Dxl(lock=20,debug=False)
    state = dxl.getPos()
    print state
    balance.execute()
    head = Head(dxl)
    rospy.init_node('Motion', anonymous=True)
    rospy.Subscriber("moco", String, moco,queue_size=1)
    feedback = rospy.Publisher('feedback', String, queue_size=1)
    while True:
        #print head.tilt_angle,head.pan_angle
        # rospy.loginfo(head.tilt_angle)
        dxl.publish(feedback)
    rospy.spin()
