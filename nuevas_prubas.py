import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
#from puzzlebot_reto1.msg import set_point
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2 
    
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
    roi_size = [int(self.roi_upper*self.alto),int(self.roi_lower*self.alto),0,self.ancho - 1]
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
        self.e = int(self.x_min - self.setpoint)/10
        self.error_d = int(self.ang - 0 + (self.e / 2.5))
        self.error_a = self.error_d
        print(self.error_d)
        if self.error_a > 4 and self.error_a<-4:
            self.error_sum_a += self.error_a * self.dt
            self.error_derivativo_a = (self.error_a - self.error_acumulado_a) / (self.dt + 0.00001)
            self.control_a = self.kp_a * self.error_a + self.ki_a * self.error_sum_a + self.kd_a * self.error_derivativo_a
            
            self.control_d = 0.2                
            #self.t1 = int(20 - 25 * np.tanh(self.control_d / 10))
            self.t1 = self.control_d
            if self.control_a>0.1 :
                self.img_out.ang = 0.05
                self.img_out.dist = self.control_d
                self.control_vel_pub.publish(self.img_out)
            
            elif self.control_a<-0.1:
                self.img_out.ang = 0.05
                self.img_out.dist = self.control_d
                self.control_vel_pub.publish(self.img_out)
            
        
            else: 
                self.img_out.ang = self.control_a
                self.img_out.dist = self.control_d
                self.control_vel_pub.publish(self.img_out)
            
            # Esta linea manda las velocidades
            print_info = "%3f | %3f" % (self.error_a, self.error_d)
            rospy.loginfo(self.img_out)

if __name__ == "__main__":
    #image_proc = img_cnontrol()
    rospy.init_node("img_cnontrol")
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            #image_proc.run()
            #image_proc.run2()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass