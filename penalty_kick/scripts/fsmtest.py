


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

if __name__ == '__main__':
    root  = State(state = 'Start',transdict={'fo':'exit','nofo':'L1'},val={'tilt':90,'pan':0})
    fsm = FSM(root)
    fsm.add(State(state='L1', transdict={'fo': 'exit', 'nofo': 'CU'}, val={'tilt': 90, 'pan': 60}))
    fsm.add(State(state='CU', transdict={'fo': 'exit', 'nofo': 'R1'}, val={'tilt': 90, 'pan': 0}))
    fsm.add(State(state='R1', transdict={'fo': 'exit', 'nofo': 'R2'}, val={'tilt': 90, 'pan': -60}))
    fsm.add(State(state='R2', transdict={'fo': 'exit', 'nofo': 'CD'}, val={'tilt': 45, 'pan': -60}))
    fsm.add(State(state='CD', transdict={'fo': 'exit', 'nofo': 'L2'}, val={'tilt': 45, 'pan': 0}))
    fsm.add(State(state='L2', transdict={'fo': 'exit', 'nofo': 'Start'}, val={'tilt': 45, 'pan': 60}))
    fsm.add(State(state='exit', val={'tilt': 90, 'pan': 0}))




    while True:
        print fsm.current,fsm.current.val
        trig = raw_input()
        fsm.update(trig)
