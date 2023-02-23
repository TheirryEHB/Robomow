
import numpy as np
import requests
from matplotlib import pyplot as plt
import time
import serial
import math
import sys
import queue

bigMatrix = []

#https://avrgeeks.com/connect-arduino-with-computer-using-hc05/#:~:text=Steps%20to%20connect%20to%20the%20Laptop&text=Once%20you%20find%20the%20HC,%E2%80%9Ccomport%E2%80%9D%20of%20the%20device.
#https://stackoverflow.com/questions/34550437/serialexception-could-not-open-port-access-is-denied
name = 'HC-05'
serverMAC = 'E0-D4-64-C1-97-E1'  # mac_bl 98:d3:31:f6:77:ff #mac_pc E0-D4-64-C1-97-E1
hostMac = '98:d3:31:f6:77:ff'
port = 11
passkey = "1234"

COM = "COM5"
#voor laptop outgoing: COM10(spp dev), incomming COM9
#laptop usb COM5
#Voor desktop outgoing: COM11(spp dev), incomming COM10
BAUD = 9600
SerialPort = serial.Serial(COM, BAUD, timeout=0.001)
time.sleep(5)

def sendData(toSend):
    SerialPort.close()
    SerialPort.open()
    SerialPort.write(str(toSend+"\n").encode("utf-8"))
    readData()

def readData():
    while 1:
        # SerialPort.close()
        # SerialPort.open()
        IncomingData = SerialPort.readline()
        if IncomingData:
            print(IncomingData.decode('utf-8'))


# sendData(str(2))
# time.sleep(5)
# readData()

while (1):
    try:
        OutgoingData=input('> ')
        SerialPort.write(bytes(OutgoingData,'utf-8'))
    except KeyboardInterrupt:
        print("Closing and exiting the program")
        SerialPort.close()
        sys.exit(0)
    IncomingData=SerialPort.readline()
    if(IncomingData):
        print((IncomingData).decode('utf-8'))
    time.sleep(0.01)
