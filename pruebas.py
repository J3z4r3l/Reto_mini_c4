import cv2
import numpy as np

class ObjectTracker:
    def _init_(self):
        self.cap = cv2.VideoCapture(0)
        self.cnts=0
        self.setpoint = None
    
    def calculate_error(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        th = cv2.inRange(roi, (0, 0, 0), (50, 50, 50))
        bordes = cv2.Canny(th, 200, 255)
        self.cnts, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.cnts = sorted(self.cnts, key=cv2.contourArea, reverse=True)[:1]

        x_min = 0
        if len(self.cnts) > 0:
            x, y, w, h = cv2.boundingRect(self.cnts[0])
            blackbox = cv2.minAreaRect(self.cnts[0])
            (x_min, y_min), (w_min, h_min), ang = blackbox

        h, w, _ = roi.shape

    def calculate_error(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        th = cv2.inRange(roi, (0, 0, 0), (50, 50, 50))
        bordes = cv2.Canny(th, 200, 255)
        self.cnts, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.cnts = sorted(self.cnts, key=cv2.contourArea, reverse=True)[:1]

        x_min = 0
        if len(self.cnts) > 0:
            x, y, w, h = cv2.boundingRect(self.cnts[0])
            blackbox = cv2.minAreaRect(self.cnts[0])
            (x_min, y_min), (w_min, h_min), ang = blackbox

        h, w, _ = roi.shape

        if self.setpoint is None:
            self.setpoint = w / 2

        lines = cv2.HoughLines(bordes, 1, np.pi / 180, 200)

        error_a = 0
        error_d = 0

        if lines is not None:
            angles = [(theta * 180 / np.pi) - 90 for line in lines for rho, theta in line]
            error_a = (self.fit_line_and_calculate_error(angles)-89)/10

        error_d = int(x_min - self.setpoint)
        if -8 < error_d < 8:
            error_d = 0

        # Limitar el error angular a -5 y 5
        if error_a < -5:
            error_a = -5
        elif error_a > 5:
            error_a = 5

        return error_a, error_d