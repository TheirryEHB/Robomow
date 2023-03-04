
import numpy as np
import requests
from matplotlib import pyplot as plt
import time
import serial
import math
import sys
import queue
from enum import Enum


# Variables
toSend = ""
bigMatrix = []
angleArray = []
class Connected(Enum):
    Connected = 1
    NotConnected = 2
connectedState = Connected.NotConnected

# https://avrgeeks.com/connect-arduino-with-computer-using-hc05/#:~:text=Steps%20to%20connect%20to%20the%20Laptop&text=Once%20you%20find%20the%20HC,%E2%80%9Ccomport%E2%80%9D%20of%20the%20device.
# https://stackoverflow.com/questions/34550437/serialexception-could-not-open-port-access-is-denied
# name = 'HC-05'
# serverMAC = 'E0-D4-64-C1-97-E1'  # mac_bl 98:d3:31:f6:77:ff #mac_pc E0-D4-64-C1-97-E1
# hostMac = '98:d3:31:f6:77:ff'
# port = 11
# passkey = "1234"

COM = "COM5"
# Voor laptop Bluetooth outgoing: COM10(spp dev), incoming COM9
# laptop usb COM5
# Voor desktop Bluetooth outgoing: COM11(spp dev), incoming COM10
BAUD = 115200
SerialPort = serial.Serial(COM,  BAUD, timeout=0.1)
connectedState = Connected.Connected
SerialPort.close()
SerialPort.open()
time.sleep(2)


def sendData():
    global toSend, SerialPort
    toSendString = str(toSend)+"\n"
    print(toSendString)
    SerialPort.write(bytes(toSendString, 'utf-8'))
    toSend = ""

def readData():
    global toSend
    while 1:
        if toSend != "":
            sendData()
        else:
            try:
                incomingData = SerialPort.readline()
                if incomingData:
                    decodedData = incomingData.decode('utf-8')
                    inArr = []
                    if "Distance Traveld: " in decodedData:
                        inArr[0] = decodedData[18:-1]
                    if "Degrees Turned: " in decodedData:
                        inArr[1] = decodedData[16:-1]
                        angleArray.append(inArr)
                    elif "Done turning." in decodedData:
                        if len(angleArray) > 10:
                            toSend = str(6)
                            break
                time.sleep(0.01)
            except KeyboardInterrupt:
                print("Closing and exiting the program")
                SerialPort.close()
                sys.exit(0)


if connectedState == Connected.Connected:
    print("Connected")
    toSend = 2
    readData()
