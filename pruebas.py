
import time
import cv2
import numpy as np

th1=0
ang=90
cap=cv2.VideoCapture(0)
x_min=0
error_a=0

while True: 
  ret,ROI = cap.read()
  gray= cv2.cvtColor(ROI,cv2.COLOR_BGR2GRAY)
  th=cv2.inRange(ROI,(0,0,0,),(50,50,50))
  bordes = cv2.Canny(th , 200,255)
  #retval, th= cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
  _,cnts, _= cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  cnts=sorted(cnts, key=cv2.contourArea,reverse=True)[:1]
  if len(cnts) > 0:
    x,y,w,h = cv2.boundingRect(cnts[0])
    blackbox = cv2.minAreaRect(cnts[0])
    (x_min,y_min), (w_min, h_min),ang = blackbox
  cv2.drawContours(ROI, cnts,0,(0,255,0),3)
  h, w, _ =ROI.shape
  setpoint = w/2
  cv2.line(ROI, (int(w/2),250),(int(w/2),280),(255,0,0),3)
  lines1 =cv2.HoughLines(bordes,1,np.pi/180,200)
  if lines1 is not None:
    for line in lines1 :
        rho,theta =line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 =int(x0 + 1000*(-b))
        y1 =int(y0 + 1000*(a))
        x2 =int(x0 - 1000*(-b))
        y2 =int(y0 - 1000*(a))
        cv2.line(th,(x1,y1),(x2,y2),(0,0,255),2,cv2.LINE_AA)
        th1=(theta * 180/np.pi)-90
  key= cv2.waitKey(1) & 0xFF
  if th1<0:  
      error_a = th1+90
  elif th1>0:
      error_a = th1-90

  error_d = int(x_min - setpoint)
  if error_d < 8 and error_d > -8:
    error_d=0

  cv2.imshow("o",ROI)
  print("error angular: ",error_a,"error de distancia: ",error_d)
  if key == ord("q"):
    break

cap.release()
cv2.destroAllWindows()

