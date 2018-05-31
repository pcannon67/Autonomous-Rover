#!/usr/bin/env python

#include cv2
import keras
import tensorflow as tf
import numpy as np
import serial
import sys
import time

arduino =  serial.Serial('COM10', 9600,timeout = 1) #connect to the arduino's serial port
time.sleep(2)

hello = tf.constant(' Hello, This is TensorFlow !')
sess = tf.Session()
welcomeMessage = sess.run(hello).decode()
print(welcomeMessage)

while(1):
    PositionSetpiont= 0
    AngleSetpoint = 0
    Velocity = 70
    
    listt = ['<'+str(PositionSetpiont),str(AngleSetpoint),str(Velocity )+'>']
    cstring = ",".join(listt)
    cstring1 = str.encode(cstring)
    
    print(cstring1)
    arduino.write(cstring1)
    time.sleep(0.001)
