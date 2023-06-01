import cv2
import numpy as np

def seg_linea(self,image):
    gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    th=cv2.inRange(roi,(0,0,0,),(50,50,50))
    retval, th= cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
    cnts, _= cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts=sorted(cnts, key=cv2.contourArea,reverse=True)[:1]
    cv2.drawContours(image, cnts,0,(0,255,0),3)
    g,b,_=image.shape
    self.setpoint = b/2
    cv2.line(image, (int(b/2),50),(int(b/2),80),(255,0,0),3)
    if len(cnts) > 0:
        x,y,w,h = cv2.boundingRect(cnts[0])
        blackbox = cv2.minAreaRect(cnts[0])
        (self.x_min,self.y_min), (self.w_min, self.h_min),ang = blackbox
        if self.ang < -45:
            self.ang = 90 + ang
        if self.w_min < self.h_min and self.ang > 0:
             self.ang = (90-ang)*-1
        if self.w_min > self.h_min and self.ang < 0:
             self.ang = 90 -ang



