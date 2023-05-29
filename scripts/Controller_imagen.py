#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from mini_c4.msg import input_point

from mini_c4.msg import imagen_data


class procesamiento:
    #inicializamos las variables para usar la camara
    def __init__(self):
        self.ancho = 640
        self.alto = 320
        #Rezise image
        self.roi_upper = 0.60
        self.roi_lower = 0.99
        self.frameWidth = 640
        self.frameHeight = 480
        self.setpoint=0.0
        #Variables para el error
        self.x_min=0.0
        self.e=0.0
        self.error_d = 0.0
        self.error_a =0.0
        self.error_acumulado_a=0.0
        self.dt=0.0
        self.kp_a = 1.0
        self.kp_d = 0.05
        self.ki_a = 0.02
        self.ki_d = 0.02
        self.kd_a = 0.003
        self.error_sum_d=0.0
        self.error_sum_a=0.0
        self.error_acumulado_a=0.0
        self.error_derivativo_a= 0.0
        self.accion_proporcional_a = 0.0
        self.accion_proporcional_d = 0.0
        self.accion_integral_a = 0.0
        self.accion_integral_d = 0.0
        self.accion_deribativa_a =0.0       
        self.control_a= 0.0
        self.control_d= 0.0
        self.t1=0.0
        self.ang=0.0
        self.image = None	
        self.bridge = cv_bridge.CvBridge()
        self.img_out= imagen_data()
        self.img_out.ang=0
        self.img_out.dist=0


	#Definir suscriptores y publicadores
        rospy.init_node("controller_img")
        self.image_sub = rospy.Subscriber('/video_source/raw',Image, self.image_callback) 
        self.control_vel = rospy.Publisher('/control_vel',imagen_data, queue_size=10)
        self.rate = rospy.Rate(10)

    #Aqui guardamos la imagen a la que nos suscribimos 
    def image_callback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    def seg_linea(self,image):
        gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #th=cv2.inRange(ROI,(0,0,0,),(50,50,50))
        retval, th= cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        cnts, _= cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts=sorted(cnts, key=cv2.contourArea,reverse=True)[:1]
        cv2.drawContours(image, cnts,0,(0,255,0),3)
        g,b,_=image.shape
        setpoint = b/2
        cv2.line(image, (int(b/2),50),(int(b/2),80),(255,0,0),3)
        
        
        if len(cnts) > 0:
            x,y,w,h = cv2.boundingRect(cnts[0])
            blackbox = cv2.minAreaRect(cnts[0])
            (self.x_min,self.y_min), (self.w_min, self.h_min),ang = blackbox
            if ang < -45:
                ang = 90 + ang
            if self.w_min < self.h_min and ang > 0:
                 ang = (90-ang)*-1
            if self.w_min > self.h_min and ang < 0:
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
        if self.first:
            self.current_time = rospy.get_time() 
            self.previous_time = rospy.get_time()
            self.first = False
        else:
            self.current_time = rospy.get_time() 
            self.dt = (self.current_time - self.previous_time)
            self.previous_time = self.current_time
        
            self.e=int(self.x_min - self.setpoint)
            self.error_d = int(self.ang - 0+(self.e/2.5))
            self.error_a =self.error_d
            if self.error_d<4 and self.error_d>-4:
                    self.error_d=0
                    self.error_sum_a += self.error_a * self.dt
                    self.error_sum_d += self.error_d * self.dt
                    self.error_derivativo_a= (self.error_a- self.error_acumulado_a) / (self.dt+0.00001)
                    self.accion_proporcional_a = self.kp_a * self.error_a
                    self.accion_proporcional_d = self.kp_d * self.error_d
                    self.accion_integral_a = self.ki_a * self.error_sum_a
                    self.accion_integral_d = self.ki_d * self.error_sum_d
                    self.accion_deribativa_a =self.kd_a * self.error_derivativo_a
                    self.control_a= self.accion_proporcional_a + self.accion_integral_a + self.accion_deribativa_a
                    self.control_d= self.accion_proporcional_d + self.accion_integral_d
                    self.t1=int(20-25*np.tanh(self.control_d/10))
                    self.img_out.ang=self.control_a
                    self.img_out.dist=self.t1
                    #esta linea manda ls velocidades
                    self.control_vel.publish(self.img_out)
                    rospy.loginfo(self.img_out)
if __name__ == "__main__":
    image_proc = procesamiento()
    while not rospy.is_shutdown():
        try:
            image_proc.run()
        except rospy.ROSInterruptException:
            None