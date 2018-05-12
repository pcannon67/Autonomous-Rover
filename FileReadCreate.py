import serial
import sys
import time

arduino =  serial.Serial('COM4', 9600,timeout = 1) #connect to the arduino's serial port
F = open("TFData.txt","w")
time.sleep(2)

while(1):
    cstring =  arduino.read()
    print(cstring)
    F.write(cstring)

F.close()
