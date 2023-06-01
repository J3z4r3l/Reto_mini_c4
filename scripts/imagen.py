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
        # Parametros para la imagen
        self.width = 640
        self.height = 320
        self.image = None
        self.bridge = cv_bridge.CvBridge()
        self.current_time = 0
        self.previous_time = 0
        self.first = False
        self.contours=0
        # Enviamos las velocidades a traves de imagen_data.msg
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
        self.prev_error_d=0
        self.prev_error_a=0
        # Parametros del controlador PID
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
        self.image_pub = rospy.Publisher('/procesada2',Image, queue_size=10)

    # Mandamos a llamar a la imagen
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Calcular el error
    def calculate_error(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        th = cv2.inRange(roi, (250, 250, 250), (255, 255, 255))
        retval, th= cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
        bordes = cv2.Canny(th, 200, 255)
        self.cnts, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.cnts = sorted(self.cnts, key=cv2.contourArea, reverse=True)[:1]
        w=0.0
        x_min = 0.0

        if len(self.cnts) > 0:
            x, y, w, h = cv2.boundingRect(self.cnts[0])
            blackbox = cv2.minAreaRect(self.cnts[0])
            (x_min, y_min), (w_min,h_min), ang = blackbox

        h, w, _ = roi.shape

        if self.setpoint is None:
            self.setpoint = w / 2

        lines = cv2.HoughLines(bordes, 1, np.pi / 180, 200)

        error_a = 0
        error_d = 0

        if lines is not None:
            angles = [(theta * 180 / np.pi) - 90 for line in lines for rho, theta in line]
            error_a = (self.fit_line_and_calculate_error(angles)-0)


        error_d = float(x_min - self.setpoint)/10
        if -10 < error_d < 10:
            error_d = 0

        elif error_d<-10:
            error_d=error_d+10
        elif error_d>10:
            error_d=error_d-10


        return error_a, error_d
    # Ajustar la linea y calcular el error 
    def fit_line_and_calculate_error(self, angles):
        histogram, _ = np.histogram(angles, bins=180, range=(-90, 90))
        error_a = histogram.argmax() - 90

        return error_a
    def create_roi(self,frame):
        rotated = cv2.rotate(frame, cv2.ROTATE_180)
        resized = cv2.resize(rotated, (self.width, self.height))
        roi_upper=0.75
        roi_lower= 0.99
        frame_width=640
        frame_height = frame.shape[0]
        n=300
        roi_size = [int(roi_upper * frame_height), int(roi_lower * frame_height), n, frame_width - 1+n]
        print(roi_size)
        roi = rotated[roi_size[0]:roi_size[1], roi_size[2]:roi_size[3]]
        #cv2.imshow("ROI", roi)
        return roi

    # Procesar el frame
    # Procesar el frame
    def process_frame(self, roi):
        roi1=self.create_roi(roi)
        self.error_a, self.error_d = self.calculate_error(roi1)
        # Calculo de los errores
        self.current_time = rospy.get_time()
        dt = (self.current_time - self.previous_time)
        self.previous_time = self.current_time

        # Termino proporcional (P)
        control_a_p = self.kp_a * self.error_d
        control_d_p = self.kp_d * self.error_d

        # Termino integral (I)
        self.error_sum_a += self.error_d * dt
        self.error_sum_d += self.error_d * dt
        control_a_i = self.ki_d * self.error_sum_d
        control_d_i = self.ki_d * self.error_sum_d

        # Termino derivativo (D)
        error_derivativo_a = (self.error_d - self.prev_error_d) / dt
        control_a_d = self.kd_a * error_derivativo_a

        # Calculo del control total
        self.control_a = control_a_p + control_a_i + control_a_d
        self.control_d = control_d_p + control_d_i

        # Actualizar variables para la siguiente iteracion
        self.prev_error_a = self.error_a
        self.prev_error_d = self.error_d

        # Envio de las velocidades al publicador correspondiente
        self.img_out.ang = self.control_a
        self.img_out.dist = 0.1#self.control_d
        self.control_vel_pub.publish(self.img_out)

        #va lo de las imagenes...
        cv2.drawContours(roi1, self.cnts, 0, (0, 255, 0), 3)
        h, w, _ = roi1.shape
        cv2.line(roi1, (int(w / 2), 120), (int(w / 2), 80), (255, 0, 0), 3)
        key = cv2.waitKey(1) & 0xFF
        #cv2.imshow("flechita", roi1)

        key = cv2.waitKey(1) & 0xFF
        print_info = "%3f | %3f| %3f" % (self.error_a, self.error_d,self.control_a)
        rospy.loginfo(self.error_d)

        return key

    def run(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                #roi = self.process_image(self.image)
                key = self.process_frame(self.image)
                #ros_img = self.bridge.cv2_to_imgmsg(self.image)
               # ros_img = self.bridge.cv2_to_imgmsg(detection_rgb)
                #self.image_pub.publish(ros_img)
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