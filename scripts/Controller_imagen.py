#!/usr/bin/env python
from ast import Pass
import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class procesamiento:
    #inicializamos las variables para usar la camara
    def __init__(self):
        self.ancho = 640
        self.alto = 320
        #Rezise image
        self.roi_upper = 0.40
        self.roi_lower = 0.99

        self.frameWidth = 640
        self.frameHeight = 480
        self.setpoint=0

        self.image = None	
        self.bridge = cv_bridge.CvBridge()

	#Definir suscriptores y publicadores
        self.image_sub = rospy.Subscriber('/video_source/raw',Image, self.image_callback) 
        self.image_pub = rospy.Publisher('/procesada',Image, queue_size=10)
        self.color_pub = rospy.Publisher('/color',Float32, queue_size=10)
    
    #Aqui guardamos la imagen a la que nos suscribimos 
    def image_callback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    def seg_linea(self,image):
        global ang , setpoint
        gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #th=cv2.inRange(ROI,(0,0,0,),(50,50,50))
        retval, th= cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        cnts, _= cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts=sorted(cnts, key=cv2.contourArea,reverse=True)[:1]
        cv2.drawContours(image, cnts,0,(0,255,0),3)
        g,b,_=image.shape
        setpoint = b/2
        cv2.line(image, (int(b/2),50),(int(b/2),80),(255,0,0),3)
        #cv2.imshow("o3",th)
        #cv2.waitKey(1)

        if len(cnts) > 0:
            x,y,w,h = cv2.boundingRect(cnts[0])
            blackbox = cv2.minAreaRect(cnts[0])
            (x_min,y_min), (w_min, h_min),ang = blackbox
            if ang < -45:
                ang = 90 + ang
            if w_min < h_min and ang > 0:
                 ang = (90-ang)*-1
            if w_min > h_min and ang < 0:
                 ang = 90 -ang
    
    def imag(self):
        image2 = cv2.resize(self.image,(self.frameWidth, self.frameHeight))
        roi_size = [int(self.roi_upper*self.frameHeight),int(self.roi_lower*self.frameHeight),0,self.frameWidth - 1]
        lineThresh = 0.1
        roi = image2[roi_size[0]:roi_size[1],roi_size[2]:roi_size[3]]
        self.seg_linea(roi)
        cv2.imshow("o2",roi)
        cv2.waitKey(1)
 
    def run(self):
        print("Hola")

if __name__ == "__main__":
    image_proc = procesamiento()
    rospy.init_node("procesamiento")
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            image_proc.run()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass