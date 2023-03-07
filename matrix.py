
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

from compas import currOrientation, changeOrLeft, changeOrRight

# Matrix Variables
toSend = ""
pingsSend = [[114, 90], [114, 90], [57, 90], [114, -90], [57, 90], [228, 90]]
headMatrix = []
angleArray = []
class Connected(Enum):
    Connected = 1
    NotConnected = 2
connectedState = Connected.NotConnected

## Pathfinding
arrayHoogte = []
arrayBasis = []
currentPosRob = [0, 0]
shortestRout = []
neighbours = []
closestNeighbour = [1, 1]
weight = 0
turnArray = []


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

##
# Make matrix step by step with data in pingsSend
##
def makematrix():
    #Step 1: from degrees to radians
    def degrees2radians(degree):
        return degree*np.pi/180

    def pinglist2radianslist(pinglist):
        return [[x[0], degrees2radians(x[1])] for x in pinglist]


    # step 2: rescale to quarter wheel turn
    def rescale(poleCords):
        poleCords = np.array(poleCords)
        scale = 57/4  # how many pings go into a 1/4 wheelturn
        poleCords[:, 0] = poleCords[:, 0]/scale
        return poleCords

    # step 3: Translate To x,y coordinates [x,y]
    def pol2cart(rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)

    # lijst van polen naar cartesysche coordinaten brengen
    def polToCart(poleCords):
        cartCords = [[0, 0]]
        for i in range(len(poleCords)):
            angle = sum([cor[1] for cor in poleCords[:i+1]]) #Som van alle hoeken voor de hoek waar we nu zitten.
            x, y = pol2cart(poleCords[i][0], angle) # x, y verkrijgen door bereking met poolcoordinaten (amplitude en hoek). Dit vanaf oorsprong perspectief
            cartCords.append([cartCords[i][0]+x, cartCords[i][1]+y]) # verplaatsen x,y (nieuwe oorsprong) naar coorinaten voorig punt door voorig punt ermee op te tellen
        cartCords.append([0, 0])
        return cartCords


    # step 4: shift axis to only positive numbers
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

    # step 5: draw to canvas
    def drawToCanvas(cartCords):
        canvas = np.zeros((max(cartCords[:, 1])+1, max(cartCords[:, 0])+1))
        [cv2.line(canvas, cartCords[i].tolist(),
                  cartCords[i+1].tolist(), 255, 1)for i in range(cartCords.shape[0]-1)]
        canvas = cv2.rotate(canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imwrite("canvas.jpg", canvas)
        print("canvas made")
        return canvas//255

    # step 6: fill in box matrix
    def fillMatrix(matrixEdge):
        # matrixEdge//255
        print("matrix edge shape: ")
        print(matrixEdge.shape)
        matrixn = np.zeros_like(matrixEdge)
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
                        matrixn[y, x] = 1
        print("Filled")
        return matrixn*255



    poleCords = pinglist2radianslist(pingsSend)
    print("polecords: ")
    print(poleCords)
    poleCords = rescale(poleCords)
    cartCords = polToCart(poleCords)
    cartCords = shiftAxis(cartCords)
    canvas = drawToCanvas(cartCords)
    matrix = fillMatrix(canvas)
    global headMatrix
    headMatrix = matrix
    cv2.imwrite("filled_matrix.jpg", matrix)
    print("Done")
    # print(matrix[3, 2])


##Matrix movement
################################
# turn degrees to right to look at right of matrix
def matrixGoRight():
    if currOrientation < 270:  # turn left until looking at right side field
        #         degreesToTurn = 270 - currOrientation
        degreesToTurn = 270 - currOrientation
        changeOrRight(360 - degreesToTurn)
        stri = "turn left: " + str(degreesToTurn)
        # print(stri)
        sendTurnData(stri)
        # ser.write(b'turn left: ' + str(degreesToTurn).encode("utf-8") + b'\n')
        # print("R turn left: " + str(degreesToTurn))
    # turn degrees to left
    elif currOrientation >= 270:
        degreesToTurn = currOrientation - 270
        changeOrRight(degreesToTurn)
        stri = "turn right: " + str(degreesToTurn)
        sendTurnData(stri)
        # ser.write(b'turn right: ' + str(degreesToTurn).encode("utf-8") + b'\n')
        # print("R turn right: " + str(degreesToTurn))
    # turn degrees to right

# turn degrees to left to look at left of matrix
def matrixGoLeft():
    if currOrientation < 90:
        degreesToTurn = 90 - currOrientation
        changeOrLeft(degreesToTurn)
        stri = "turn right: " + str(degreesToTurn)
        sendTurnData(stri)
        # ser.write(b'turn right: ' + str(degreesToTurn).encode("utf-8") + b'\n')
        # print("L turn right: " + str(degreesToTurn))
    # turn degrees to right
    elif currOrientation >= 90:
        degreesToTurn = currOrientation - 90
        changeOrLeft(degreesToTurn)
        stri = "turn left: " + str(degreesToTurn)
        sendTurnData(stri)
        # ser.write(b'turn left: ' + str(degreesToTurn).encode("utf-8") + b'\n')
        # print("L turn left: " + str(degreesToTurn))

# turn degrees to left to look at front of matrix
def matrixGoForward():
    if currOrientation > 45 and currOrientation < 315:
        degreesToTurn = 360 - currOrientation
        changeOrLeft(degreesToTurn)
        stri = "turn left: " + str(degreesToTurn)
        sendTurnData(stri)
        # ser.write(b'turn left: ' + str(degreesToTurn).encode("utf-8") + b'\n')
    else:
        changeOrLeft(0)
        stri = "turn left: " + str(0)
        sendTurnData(stri)
        # ser.write(b'turn left: ' + b'0' + b'\n')
        # print("turn hella left: " + str(degreesToTurn))

# turn degrees to left to look at back of matrix
def matrixGoBack():
    if currOrientation < 180:
        degreesToTurn = 180 - currOrientation
        changeOrLeft(degreesToTurn)
        stri = "turn left: " + str(degreesToTurn)
        sendTurnData(stri)
        # print("B turn left: " + str(degreesToTurn))
        # turn degrees to left
    elif currOrientation >= 180:
        degreesToTurn = currOrientation - 180
        changeOrLeft(degreesToTurn)
        stri = "turn right: " + str(degreesToTurn)
        sendTurnData(stri)
        # print("B turn right: " + str(degreesToTurn))
        # turn degrees to right

##
# Calculate the next direction Robomow should ride
##
def calcMatrixGoDirection(pointToGo):
    #     print(currentPosRob)
    if pointToGo[1] == currentPosRob[1]:  # go forward or back
        if pointToGo[0] > currentPosRob[0]:
            print("Forward")
            matrixGoForward()
        elif pointToGo[0] < currentPosRob[0]:
            print("Back")
            matrixGoBack()
    if pointToGo[0] == currentPosRob[0]:  # turn left or right
        if pointToGo[1] > currentPosRob[1]:
            print("Right")
            matrixGoRight()
        if pointToGo[1] < currentPosRob[1]:
            print("Left")
            matrixGoLeft()

##
# add next direction to array
##
def sendTurnData(strr):
    turnArray.append(strr)

##
# Send next direction to go to Robomow
##
def nextDirections():
    if (len(turnArray) > 0):
        # print(turnArray[0])
        # blu_send.send((turnArray[0]+"\r").encode())
        turnArray.pop(0)



##Shortest Route
######################
# https://www.lavivienpost.com/shortest-path-between-cells-in-matrix-code/#:~:text=Shortest%20path%20in%20matrix%20is,starts%20from%20the%20source%20node.
##
# Class of a cell object for shortest rout calculation
##
class Cell:
    def __init__(self, x, y, dist, prev):
        self.x = x
        self.y = y
        self.dist = dist  # distance to start
        self.prev = prev  # parent cell in the path

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"

##
# Calculate shortest route with BFS
##
class ShortestPathBetweenCellsBFS:
    # BFS, Time O(n^2), Space O(n^2)
    def shortestPath(self, matrix, start, end):
        sx = start[0]
        sy = start[1]
        dx = end[0]
        dy = end[1]
        # if start or end value is 0, return
        if matrix[sx][sy] == 0 or matrix[dx][dy] == 0:
            print("There is no path.")
            return
        # initialize the cells
        m = len(matrix)
        n = len(matrix[0])
        cells = []
        for i in range(0, m):
            row = []
            for j in range(0, n):
                if matrix[i][j] != 0:
                    row.append(Cell(i, j, sys.maxsize, None))
                else:
                    row.append(None)
            cells.append(row)
            # breadth first search
        queue = []
        src = cells[sx][sy]
        src.dist = 0
        queue.append(src)
        dest = None
        p = queue.pop(0)
        while p != None:
            # find destination
            if p.x == dx and p.y == dy:
                dest = p
                break
                # moving up
            self.visit(cells, queue, p.x - 1, p.y, p)
            # moving left
            self.visit(cells, queue, p.x, p.y - 1, p)
            # moving down
            self.visit(cells, queue, p.x + 1, p.y, p)
            # moving right
            self.visit(cells, queue, p.x, p.y + 1, p)
            if len(queue) > 0:
                p = queue.pop(0)
            else:
                p = None
                # compose the path if path exists
        if dest == None:
            print("there is no path.")
            return
        else:
            path = []
            p = dest
            while p != None:
                path.insert(0, p)
                p = p.prev
            for i in path:
                shortestRout.append([i.x, i.y])

    # function to update cell visiting status, Time O(1), Space O(1)
    def visit(self, cells, queue, x, y, parent):
        # out of boundary
        if x < 0 or x >= len(cells) or y < 0 or y >= len(cells[0]) or cells[x][y] == None:
            return
        # update distance, and previous node
        dist = parent.dist + 1
        p = cells[x][y]
        if dist < p.dist:
            p.dist = dist
            p.prev = parent
            queue.append(p)

##
# Calculate and show shortest route
##
def calcShortestRoute():
    global currentPosRob
    myObj = ShortestPathBetweenCellsBFS()
    # case1, there is no path
    start = [3, 5]
    currentPosRob = start
    end = [4, 6]
    print(headMatrix)
    # print(headMatrix[1])
    print("case 1: ")
    myObj.shortestPath(headMatrix, start, end)
    print(shortestRout)
    for i in shortestRout:
        if i != currentPosRob:
            print("case: " + str(currentPosRob) + " To " + str(i))
            calcMatrixGoDirection(i)
            currentPosRob = i
            # print(currentPosRob)
            # print(currOrientation)
            nextDirections()

makematrix()
calcShortestRoute()

## Communication
###########################
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

# if connectedState == Connected.Connected:
#     print("Connected")
#     toSend = 2
#     readData()
