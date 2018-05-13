import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
   ret , frame = cap.read()
   gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
   #rows,cols,channels = gray.shape()
   roi = frame
   
   ret, mask = cv2.threshold(gray,200,255,cv2.THRESH_BINARY_INV)
   cv2.line(frame,(0,0),(150,150),(0,255,255),15)
   cv2.rectangle(frame,(200,100),(500,350),(0,0,255),5)
   font = cv2.FONT_HERSHEY_SIMPLEX
   cv2.putText(frame,'Arsenal Are A Disgrace!!!',(170,200), font, 1,(0,0,0),5)
   cv2.imshow('frame',frame)
   cv2.imshow('mask',mask)
   cv2.imshow('grey',gray)
   pix = frame[300,150]
   roi = frame[200:500,100:350]
   print(pix)
   #print(roi)
   if cv2.waitKey(1) & 0xFF == ord('q'):
       break

cap.release()
cv2.destroyAllWindows()

#img = cv2.imread('watch.jpg',cv2.IMREAD)
#cv2.line(img,(0,0),(150,150),(255,255,255),15)
#cv2.imshow('img',img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
