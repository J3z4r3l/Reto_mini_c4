#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from pssy_cat.msg import imagen_data

class ImageControl:
    def __init__(self):
        # Parámetros para la imagen
        self.width = 640
        self.height = 320
        self.roi_upper = 0.60
        self.roi_lower = 0.99
        self.image = None
        self.bridge = cv_bridge.CvBridge()
        self.current_time = 0 
        self.previous_time = 0
        self.first = False
            
        # Enviamos las velocidades a través de imagen_data.msg
        self.img_out = imagen_data()
        self.img_out.ang = 0
        self.img_out.dist = 0
        
        # Variables para el controlador PID
        self.setpoint = None
        self.error_a = 0.0
        self.error_d = 0.0
        self.error_sum_a = 0.0
        self.error_sum_d = 0.0
        self.prev_error_d=0
        self.prev_error_a=0
        # Parámetros del controlador PID
        self.kp_a = 0.01
        self.ki_a = 0.00
        self.kd_a = 0.000
        self.kp_d = 0.01
        self.ki_d = 0.00
        self.control_a = 0.0
        self.control_d = 0.0
        
        # Definir suscriptores y publicadores
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback) 
        self.control_vel_pub = rospy.Publisher('/control_vel', imagen_data, queue_size=10)
        
    # Mandamos a llamar a la imagen
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # Procesamos la imagen 
    def process_image(self, image):
        rotated = cv2.rotate(image, cv2.ROTATE_180)
        resized = cv2.resize(rotated, (self.width, self.height))
        roi_size = [int(self.roi_upper * self.height), int(self.roi_lower * self.height), 0, self.width - 1]
        roi = resized[roi_size[0]:roi_size[1], roi_size[2]:roi_size[3]]
        
        self.calculate_error(roi)
        
        cv2.imshow("o2", roi)
        cv2.waitKey(1)
        
        return roi
    
    # Calcular el error
    def calculate_error(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        thresholded = cv2.inRange(roi, (0, 0, 0), (50, 50, 50))
        edges = cv2.Canny(thresholded, 250, 255)
        _, contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        x_min = 0
        if len(contours) > 0:
            x, y, w, h = cv2.boundingRect(contours[0])
            blackbox = cv2.minAreaRect(contours[0])
            (x_min, y_min), (w_min, h_min), ang = blackbox

        h, w, _ = roi.shape

        if self.setpoint is None:
            self.setpoint = w / 2

        lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

        error_a = 0
        error_d = 0

        if lines is not None:
            angles = [(theta * 180 / np.pi) - 90 for line in lines for rho, theta in line]
            error_a = self.fit_line_and_calculate_error(angles)

        error_d = int(x_min - self.setpoint)
        if -8 < error_d < 8:
            error_d = 0

        self.error_a = error_a
        self.error_d = error_d

    # Ajustar la línea y calcular el error 
    def fit_line_and_calculate_error(self, angles):
        histogram, _ = np.histogram(angles, bins=180, range=(-90, 90))
        error_a = histogram.argmax() - 90

        return error_a
    
    # Procesar el frame
    def process_frame(self, roi):
        if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
        else:

            self.calculate_error(roi)
            # Calculo de los errores
            self.current_time = rospy.get_time() 
            dt = (self.current_time - self.previous_time)
            self.previous_time = self.current_time
                
            # Termino proporcional (P)
            control_a_p = self.kp_a * self.error_a
            control_d_p = self.kp_d * self.error_d

            # Termino integral (I)
            self.error_sum_a += self.error_a * dt
            self.error_sum_d += self.error_d * dt
            control_a_i = self.ki_a * self.error_sum_a
            control_d_i = self.ki_d * self.error_sum_d

            # Termino derivativo (D)
            error_derivativo_a = (self.error_a - self.prev_error_a) / dt
            control_a_d = self.kd_a * error_derivativo_a

            # Calculo del control total
            self.control_a = control_a_p + control_a_i + control_a_d
            self.control_d = control_d_p + control_d_i

            # Actualizar variables para la siguiente iteracion
            self.prev_error_a = self.error_a
            self.prev_error_d = self.error_d

            # Envío de las velocidades al publicador correspondiente
            self.img_out.ang = self.control_a
            self.img_out.dist = self.control_d
            self.control_vel_pub.publish(self.img_out)

            # Resto del código existente
            cv2.drawContours(roi, self.cnts, 0, (0, 255, 0), 3)
            h, w, _ = roi.shape
            cv2.line(roi, (int(w / 2), 250), (int(w / 2), 280), (255, 0, 0), 3)
            key = cv2.waitKey(1) & 0xFF
            print_info = "%3f | %3f" % (self.error_a, self.error_d)
            rospy.loginfo(print_info)
            cv2.imshow("o", roi)

            return key

    def run(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                roi = self.process_image(self.image)
                key = self.process_frame(roi)

                if key == ord("q"):
                    break

        cv2.destroyAllWindows()


if __name__ == "__main__":
    image_proc = ImageControl()
    rospy.init_node("image_control")
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            image_proc.run()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass
