import cv2
import time
import numpy as np

prev = 0
cxg,cyg,cxo,cyo = 0,0,0,0
cap = cv2.VideoCapture(1)
w,h = cap.get(3),cap.get(4)
 

def geth(obj):
    return obj.h

class Obstacle():
    def __init__(self,x,y,w,h,color):
        self.x,self.y,self.w,self.h = x,y,w,h 
        self.color = color

class Obstacles():
    def __init__(self,reds,blues):
        self.obs = reds + blues
	self.reds = reds
        #self.yellows = yellows
        self.blues = blues
        self.sort()

    def sort(self):
        self.obs = sorted(self.obs,key=geth)
        #print self.obs

class vision:
    def __init__(self,low,high):
        self.low = low
        self.high = high
        self.color = self.getcolor()
	
    def getcolor(self):
        global r1,r2,b1,b2,y1,y2
        if self.low == r1 and self.high == r2:
            return 'red'
	elif self.low == y1 and self.high == y2:
	    return 'yellow'
        elif self.low == b1 and self.high == b2:
            return 'blue'

    def objects(self,hsv):
        obs = []
        
        mask = cv2.inRange(hsv, (np.array([self.low[0],self.low[1],self.low[2]])), (np.array([self.high[0],self.high[1],self.high[2]])))
        erode = cv2.erode(mask,None,iterations = 4)
        dilate = cv2.dilate(erode,None,iterations = 10)
        contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            #c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) > 10:
                x, y, w, h = cv2.boundingRect(cnt)
            #print x, y, w, h
                obs.append(Obstacle(x,y,w,h,self.color))
        return obs

ran = 30
r1 = [85-ran,191-ran,104-ran]
r2 = [85+ran,191+ran,104+ran]
y1 = [200-ran,133-ran,46-ran]
y2 = [200+ran,133+ran,46+ran]
b1 = [83-ran,108-ran,172-ran]
b2 = [83+ran,108+ran,172+ran]


red = vision(r1,r2)
yellow = vision(y1,y2)
blue = vision(b1,b2)

while True:
    _,f = cap.read()
    f = cv2.flip(f,1)
    blur = cv2.medianBlur(f,3)
    '''
    img_yuv = cv2.cvtColor(blur, cv2.COLOR_BGR2YUV)
    img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
    img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
    '''
    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2YUV)
    #hsv[:,:,2] = cv2.equalizeHist(hsv[:,:,2])
    #cv2.imshow("evr",hsv)
    reds = red.objects(hsv)
    for a in reds:
        #print geth(a)
	cv2.rectangle(f,(a.x,a.y),(a.x+a.w,a.y+a.h),[0,0,255],2)
        pass
    
    blues = blue.objects(hsv)
    for a in blues:
        #print a.x,a.y,a.w,a.h,a.color
	cv2.rectangle(f,(a.x,a.y),(a.x+a.w,a.y+a.h),[255,0,0],2)
        pass
    yellows = yellow.objects(hsv)
    for a in yellows:
        cv2.rectangle(f,(a.x,a.y),(a.x+a.w,a.y+a.h),[0,255,255],2)
        pass
    '''
    obstacles = Obstacles(reds,blues)
    for x in obstacles.obs:
        print x.h,x.color,
    '''
    cv2.imshow("f",f)
    cv2.waitKey(25) 
    print "\n\n"



    
