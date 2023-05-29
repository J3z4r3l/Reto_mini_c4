#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from mini_c4.msg import input_point
import numpy as np


class path_g:
    def __init__(self):
        # Parametros fisicos del robot
        self.l = 0.19 #Quizas estos los puedo cambiar por parametros?
        self.r = 0.05
        #velocidades de las llantas
        self.wr = 0.0
        self.wl = 0.0
        #Posicion del robot
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0
        #tiempo 
        self.current_time = 0.0
        self.previous_time = 0.0
        #bandera????????????
        self.first = True
        #errores que son mensajes 
        self.error= input_point()
        self.error.ed=0.0
        self.error.eang=0.0
        #puntos
        self.points_org=0
        self.points_goal=0
        self.listax=0
        self.listay=0
        self.i=0

        #incializamos el nodo y creamos los topicos
        rospy.init_node("path_generator")
        rospy.Subscriber("/wr",Float32,self.wr_callback)
        rospy.Subscriber("/wl",Float32,self.wl_callback)
        self.pose_pub = rospy.Publisher("/pose",Pose2D,queue_size=1)
        self.error_pub= rospy.Publisher("/error",input_point,queue_size=1)
        self.listax=rospy.get_param("list_of_x",[1.0, 0.0, 1.0, 1.0])
        self.listay=rospy.get_param("list_of_y",[1.0, 2.0, 2.5, 2.5])
            
        self.loop_rate = rospy.Rate(20)
        rospy.on_shutdown(self.stop)

    def run(self):

        #try:
        while not rospy.is_shutdown():
            # Compute time since last main loop
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
            else:
                self.current_time = rospy.get_time()
                dt = (self.current_time - self.previous_time)
                self.previous_time = self.current_time
    
                # Actualizar las posiciones
                self.pose.theta += self.wrap_to_pi(dt * self.r * ((self.wr - self.wl) / self.l))
                self.pose.x += dt * self.r * ((self.wr + self.wl) / 2) * np.cos(self.pose.theta)
                self.pose.y += dt * self.r * ((self.wr + self.wl) / 2) * np.sin(self.pose.theta)
                #puntos 
                self.points_org=np.array([self.pose.x, self.pose.y])
                #erorres
                self.error.eang = self.wrap_to_pi(np.arctan2(self.listay[self.i]-self.pose.y, self.listax[self.i]-self.pose.x) - self.pose.theta)
                self.error.ed = np.sqrt(np.square(self.listax[self.i]-self.pose.x) + np.square(self.listay[self.i]-self.pose.y))
                # Publicar las posiciones y el error
                if self.error.ed < 0.02:
                    self.error.ed=0
                if self.error.eang < 0.02 and self.error.eang > -0.02:
                    self.error.eang=0
                if self.error.eang==0 and self.error.ed==0 and self.i<len(self.listax):
                    self.i+=1
                    print("Cambiamos de punto")
                if self.i==len(self.listax):
                    print("Se acabo")
                    self.error.ed=0
                    self.error.eang=0
                print_info = "%3f | %3f" %(self.listax[self.i],self.listay[self.i])
                rospy.loginfo(dt)
                self.pose_pub.publish(self.pose)
                self.error_pub.publish(self.error)
                self.loop_rate.sleep()
               
    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data
    #funcion que regresa py??? pero limitandolo 
    def wrap_to_pi(self,theta):
        result=np.fmod((theta+ np.pi),(2*np.pi))
        if(result<0):
            result += 2 * np.pi
        return result - np.pi
    def stop(self):
        print("Stopping")

if __name__ == "__main__":
    controller = path_g()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        None