#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from mini_c4.msg import input_point
 
class Controller:
    def __init__(self):
        # Configurar parametros del controlador
        self.kp_l = 0.1 
        self.ki_l = 0.00
        self.kd_l = 0.0 
        self.kp_ang = 1.0
        self.ki_ang = 0.01
        self.kd_ang = 0.1


        self.error_sum_l = 0.0
        self.error_prev_l = 0.0
        self.error_diff_l = 0.0
        self.error_sum_ang = 0.0
        self.error_prev_ang = 0.0
        self.error_diff_ang = 0.0
        self.error_dist=0.0
        self.error_ang=0.0
        self.first=True
        self.controlador_vl=0.0
        self.controlador_va=0.0  
        self.color=0.0      

        #Inicializar nodos
        rospy.init_node("controller")
        rospy.Subscriber("/wr",Float32,self.wr_callback)
        rospy.Subscriber("/wl",Float32,self.wl_callback)
        rospy.Subscriber("/error",input_point,self.callback)
        rospy.Subscriber('/color',Float32, self.color_callback)   
        self.pose_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        
        self.rate = rospy.Rate(10)

    # Callbacks para velocidades de las ruedas
    def color_callback(self, msg):
        self.color= msg.data

    def callback(self, msg):
        self.error_dist=msg.ed
        self.error_ang=msg.eang

    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data
    
    def run(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        while not rospy.is_shutdown():
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
            else:
                self.current_time = rospy.get_time() 
                dt = (self.current_time - self.previous_time)
                self.previous_time = self.current_time
                #Controlador lineal  
                self.error_sum_l += self.error_dist * dt
                self.error_diff_l = (self.error_dist - self.error_prev_l) / (dt+0.000000001)
                self.error_prev_l = self.error_dist
                self.controlador_vl= self.kp_l * self.error_dist + self.ki_l * self.error_sum_l + self.kd_l * self.error_diff_l
                #controlador angular
                self.error_sum_ang += self.error_ang * dt
                self.error_diff_ang = (self.error_ang - self.error_prev_ang) / (dt+0.0000001)
                self.error_prev_ang = self.error_ang
                self.controlador_va =self.kp_ang * self.error_ang + self.ki_ang * self.error_sum_ang + self.kd_ang * self.error_diff_ang
                #Publicar las posiciones
                if self.color==1:
                     msg.linear.x = 0
                     msg.angular.z = 0    
                elif self.color==2:
                    msg.linear.x = self.controlador_vl/2
                    msg.angular.z = self.controlador_va/2
                elif self.color==3:
                    msg.linear.x = self.controlador_vl
                    msg.angular.z = self.controlador_va
                #Esta linea parece no servir, la quitare?
                if self.error_ang==0 and self.error_dist==0:
                    msg.linear.x = 0
                    msg.angular.z = 0
                print("Controlador_on")
                print_info = "%3f | %3f" %(self.color,msg.linear.x)
                rospy.loginfo(print_info)
                self.pose_pub.publish(msg)
                self.rate.sleep()
    def stop(self):
        print("Stopping")
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.pose_pub.publish(self.msg)

if __name__ == "__main__":
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        None