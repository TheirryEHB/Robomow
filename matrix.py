
import numpy as np
import requests
from matplotlib import pyplot as plt
import time
import serial
import math
import sys
import queue
from enum import Enum
import cv2


# Variables
toSend = ""
pingsSend = []
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


def makematrix():
    def degrees2radians(degree):
        return degree*np.pi/180

    def pinglist2radianslist(pinglist):
        return [[x[0], degrees2radians(x[1])] for x in pinglist]


    # step 4: rescale to quarter wheel turn
    def rescale(poleCords):
        poleCords = np.array(poleCords)
        scale = 57/4  # how many pings go into a 1/4 wheelturn
        poleCords[:, 0] = poleCords[:, 0]/scale
        return poleCords

    # step 5: Translate To x,y coordinates [x,y]
    def pol2cart(rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)

    ## lijst van polen naar cartesysche coordinaten brengen
    def polToCart(poleCords):
        cartCords = [[0, 0]]
        for i in range(len(poleCords)):
            angle = sum([cor[1] for cor in poleCords[:i+1]]) #Som van alle hoeken voor de hoek waar we nu zitten.
            x, y = pol2cart(poleCords[i][0], angle) # x, y verkrijgen door bereking met poolcoordinaten (amplitude en hoek). Dit vanaf oorsprong perspectief
            cartCords.append([cartCords[i][0]+x, cartCords[i][1]+y]) # verplaatsen x,y (nieuwe oorsprong) naar coorinaten voorig punt door voorig punt ermee op te tellen 
        cartCords.append([0, 0])
        return cartCords


    # step 6: shift axis to only positive numbers
    # Bepalen nullpunt van matrix zodat Y en x as enkel positieve waarden hebben
    # cartCords = np.array([[0, 0], [30, 0], [20, 10], [30, 15], [5, 15], [10, 5], [
    #  0, 0]])
    def shiftAxis(cartCords):
        cartCords = np.array(np.round(cartCords), dtype=int)
        x_axis = cartCords[:, 0]
        y_axis = cartCords[:, 1]
        # x-axis
        if any(x_axis < 0):
            cartCords[:, 0] -= min(x_axis)
        # y-axis
        if any(y_axis < 0):
            cartCords[:, 1] -= min(y_axis)
        return cartCords

    # step 7: draw to canvas
    def drawToCanvas(cartCords):
        canvas = np.zeros((max(cartCords[:, 1])+1, max(cartCords[:, 0])+1))
        [cv2.line(canvas, cartCords[i].tolist(),
                  cartCords[i+1].tolist(), 255, 1)for i in range(cartCords.shape[0]-1)]
        canvas = cv2.rotate(canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imwrite("canvas.jpg", canvas)
        return canvas

    # step 8: fill in box matrix
    def fillMatrix(matrixEdge):
        matrixEdge//255
        print(matrixEdge.shape)
        matrix = np.zeros_like(matrixEdge)
        for y in range(1, matrixEdge.shape[0]-1):
            for x in range(1, matrixEdge.shape[1]-1):
                if matrixEdge[y, x] != 1:
                    inside = 0
                    for i in range(x, -1, -1):
                        if matrixEdge[y, i] == 1:
                            inside += 1
                            break
                    for i in range(x, matrixEdge.shape[1]):
                        if matrixEdge[y, i] == 1:
                            inside += 1
                            break
                    for i in range(y, -1, -1):
                        if matrixEdge[i, x] == 1:
                            inside += 1
                            break
                    for i in range(y, matrixEdge.shape[0]):
                        if matrixEdge[i, x] == 1:
                            inside += 1
                            break
                    if inside == 4:
                        matrix[y, x] = 1
        return matrix*255



    poleCords = pinglist2radianslist(pingsSend)
    print(poleCords)
    poleCords = rescale(poleCords)
    cartCords = polToCart(poleCords)
    cartCords = shiftAxis(cartCords)
    canvas = drawToCanvas(cartCords)
    matrix = fillMatrix(canvas)
    cv2.imwrite("filled_matrix.jpg", matrix)





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
