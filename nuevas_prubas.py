import cv2
import numpy as np

class ObjectTracker:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cnts=0
        self.setpoint = None
    def calculate_error(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        th = cv2.inRange(roi, (0, 0, 0), (50, 50, 50))
        bordes = cv2.Canny(th, 250, 255)
        _, self.cnts, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
            error_a = self.fit_line_and_calculate_error(angles)

        error_d = int(x_min - self.setpoint)
        if -8 < error_d < 8:
            error_d = 0

        return error_a, error_d

    def fit_line_and_calculate_error(self, angles):
        histogram, _ = np.histogram(angles, bins=180, range=(-90, 90))
        error_a = histogram.argmax() - 90

        return error_a
    def create_roi(self,frame):
        roi_upper=0.75
        roi_lower= 0.99
        frame_width=640
        frame_height = frame.shape[0]
        roi_size = [int(roi_upper * frame_height), int(roi_lower * frame_height), 0, frame_width - 1]
        print(roi_size)
        roi = frame[roi_size[0]:roi_size[1], roi_size[2]:roi_size[3]]
        cv2.imshow("ROI", roi)
        return roi

    def run(self):
        while True:
            ret, roi1 = self.cap.read()
            roi=self.create_roi(roi1)
            error_a, error_d = self.calculate_error(roi)

            cv2.drawContours(roi, self.cnts, 0, (0, 255, 0), 3)
            h, w, _ = roi.shape

            cv2.line(roi, (int(w / 2), 150), (int(w / 2), 80), (255, 0, 0), 3)

            key = cv2.waitKey(1) & 0xFF

            print("error angular:", error_a, "error de distancia:", error_d)

            cv2.imshow("o", roi)

            if key == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    tracker = ObjectTracker()
    tracker.run()