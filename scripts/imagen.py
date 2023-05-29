#!/usr/bin/env python

from ast import Pass
import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from pssy_cat.msg import imagen_data

class procesamiento:
    def __init__(self):
        self.ancho = 640
        self.alto = 320
        #parte nueva 
        #Rezise image
        self.roi_upper = 0.60
        self.roi_lower = 0.99
        self.frameWidth = 640
        self.frameHeight = 320
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
        self.img_out= imagen_data()
        self.img_out.ang=0
        self.img_out.dist=0
        self.first= True


#finish
        self.image = None	
        self.bridge = cv_bridge.CvBridge()
        self.green_time = 0
        self.red_time = 0
        self.yellow_time = 0
        self.color_imagen=0.00

	#Definir suscriptores y publicadores
        self.image_sub = rospy.Subscriber('/video_source/raw',Image, self.image_callback) 

        self.image_pub = rospy.Publisher('/procesada',Image, queue_size=10)
        self.color_pub = rospy.Publisher('/color',Float32, queue_size=10)
        self.control_vel = rospy.Publisher('/control_vel',imagen_data, queue_size=10)
        
    def image_callback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    #Creo no ocupo esta funcion 
    def detect_traffic_light(self,image):
        rotar = cv2.rotate(image,cv2.ROTATE_180)
        redimensionar = cv2.resize(rotar,(self.ancho, self.alto))
        frame = np.copy(redimensionar)  # Copiar la imagen original
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=50, param2=30,
                               minRadius=50, maxRadius=70)
        if circles is not None:
            #self.color_pub=0
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                color = frame[i[1], i[0]]
                if color[2] > 100 and color[1] < 100 and color[0] < 100:
                    cv2.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
                    cv2.putText(frame, 'rojo', (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), lineType=cv2.LINE_AA)
                    self.yellow_time = 0
                    self.green_time = 0
                    self.red_time += 1
                    self.color_imagen=1
                elif color[2] > 100 and color[1] > 100 and color[0] < 100:
                    cv2.circle(frame,(i[0],i[1]),i[2],(0,255,255),2)
                    cv2.putText(frame, 'amarillo', (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), lineType=cv2.LINE_AA)
                    self.yellow_time += 1
                    self.red_time = 0
                    self.green_time = 0
                    self.color_imagen=2
                elif color[1] > color[0] and color[1] > color[2] and color[0] < 100 and color[2] < 100:
                    cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
                    cv2.putText(frame, 'verde', (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
                    self.green_time += 1
                    self.red_time = 0
                    self.yellow_time = 0
                    self.color_imagen=3
                else:
                    pass
        self.color_pub.publish(self.color_imagen)
        #rospy.loginfo(self.color_imagen)
        #cv2.imshow('Circulos detectados', frame)
        #cv2.waitKey(1)
        return frame

    def seg_linea(self,image):
        gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

        #th=cv2.inRange(ROI,(0,0,0,),(50,50,50))
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
        



    

    def imag(self,image):
        rotar = cv2.rotate(image,cv2.ROTATE_180)
        image2 = cv2.resize(rotar,(self.ancho, self.alto))
        roi_size = [int(self.roi_upper*self.frameHeight),int(self.roi_lower*self.frameHeight),0,self.frameWidth - 1]
        lineThresh = 0.1
        roi = image2[roi_size[0]:roi_size[1],roi_size[2]:roi_size[3]]
        self.seg_linea(roi)
        cv2.imshow("o2",roi)
        cv2.waitKey(1)
        print('corre')
        return roi
    def run(self):
        if self.image is None:
            return
        if self.first:
            self.current_time = rospy.get_time()
            self.previous_time = rospy.get_time()
            self.first = False
        else:
            self.imag(self.image)
            self.current_time = rospy.get_time()
            self.dt = (self.current_time - self.previous_time)
            self.previous_time = self.current_time

            self.e = int(self.x_min - self.setpoint)
            self.error_d = int(self.ang - 0 + (self.e / 2.5))
            self.error_a = self.error_d
	    #print(self.error_d)
            if self.error_d > -4:
                #self.error_d = 0
                self.error_sum_a += self.error_a * self.dt
                self.error_sum_d += self.error_d * self.dt
                self.error_derivativo_a = (self.error_a - self.error_acumulado_a) / (self.dt + 0.00001)
                self.accion_proporcional_a = self.kp_a * self.error_a
                self.accion_proporcional_d = self.kp_d * self.error_d
                self.accion_integral_a = self.ki_a * self.error_sum_a
                self.accion_integral_d = self.ki_d * self.error_sum_d
                self.accion_deribativa_a = self.kd_a * self.error_derivativo_a
                self.control_a = self.accion_proporcional_a + self.accion_integral_a + self.accion_deribativa_a
                self.control_d = self.accion_proporcional_d + self.accion_integral_d
                self.t1 = int(20 - 25 * np.tanh(self.control_d / 10))
                self.img_out.ang = self.control_a
                self.img_out.dist = self.t1
                # Esta linea manda las velocidades
                self.control_vel.publish(self.img_out)
                print_info = "%3f | %3f" % (self.error_a, self.error_d)
                rospy.loginfo(print_info)

        detection = self.detect_traffic_light(self.image)

        if detection is not None:
            detection_rgb = cv2.cvtColor(detection, cv2.COLOR_BGR2RGB)
            ros_img = self.bridge.cv2_to_imgmsg(detection_rgb)
            # self.image_pub.publish(ros_img)
   
if __name__ == "__main__":
    image_proc = procesamiento()
    rospy.init_node("procesamiento")
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            image_proc.run()
            #image_proc.run2()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass