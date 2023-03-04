
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
# Voor laptop outgoing: COM10(spp dev), incomming COM9
# laptop usb COM5
# Voor desktop outgoing: COM11(spp dev), incomming COM10
BAUD = 115200
SerialPort = serial.Serial('/dev/ttyACM0',  baudrate=BAUD, timeout=0.1)
connectedState = Connected.Connected
# time.sleep(10)
# SerialPort.write(bytes("2", 'utf-8'))

def sendData():
    global toSend, SerialPort
    SerialPort.close()
    SerialPort.open()
    toSendString = str(toSend)+"\n"
    print(toSendString)
    # SerialPort.write(toSendString.encode("utf-8"))
    SerialPort.write(bytes(toSendString, 'utf-8'))
    toSend = ""
    # SerialPort.close()

def readData():
    global toSend
    while 1:
        if toSend != "":
            sendData()
        else:
            try:
                # SerialPort.open()
                incomingData = SerialPort.readline()
                if incomingData:
                    print(incomingData)
                    decodedData = incomingData.decode('utf-8')
                    print(decodedData)
                    if "Degrees Turned: " in decodedData:
                        print(decodedData[15:-1])
                        angleArray.append(decodedData[15:-1])
                    elif "Done turning." in decodedData:
                        toSend = str(6)
                        break

                time.sleep(0.01)
                # OutgoingData=input('> ')
                # SerialPort.write(bytes("OutgoingData",'utf-8'))
                # SerialPort.write(bytes("2",'utf-8'))
            except KeyboardInterrupt:
                print("Closing and exiting the program")
                SerialPort.close()
                sys.exit(0)
        # SerialPort.close()


if connectedState == Connected.Connected:
    # global toSend
    print("Connected")

    # sendData(str(2))
    # time.sleep(5)
    toSend = 2
    readData()
