import cv2
import sys
import numpy as np

cap = cv2.VideoCapture(1)
F = open("EdgeData.txt","w")

while True:
   ret , frame = cap.read()
   #ret = boolen true or false, true when video is being captured
   gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
   
   edges = cv2.Canny(gray,100,300);
   #edge detection
   print(edges)
   F.write(str(edges))
   
   cv2.line(frame,(350,100),(350,350),(0,0,0),5)
   cv2.line(frame,(200,225),(500,225),(0,0,0),5)
   cv2.rectangle(frame,(200,100),(500,350),(0,0,255),5)
   cv2.circle(frame,(350,225),100,(255,255,255),5)
   # use a width of -1 to fill shapes in
   
   font = cv2.FONT_HERSHEY_SIMPLEX
   cv2.putText(frame,'Target Acquired',(230,400), font, 1,(255,255,255),2)
   
   cv2.imshow('frame',frame)
   cv2.imshow('edges',edges)
   cv2.imshow('gray',gray)
  
   if cv2.waitKey(1) & 0xFF == ord('q'):
       break
      
F.close()
cap.release()
#out.release()
cv2.destroyAllWindows()

#cv2.waitKey(0)
