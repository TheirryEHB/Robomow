import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import serial
import math

print(cv2.__version__)

detector = cv2.QRCodeDetector()

#https://www.instructables.com/Raspberry-Pi-Arduino-Serial-Communication/
#https://www.arrow.com/en/research-and-events/articles/raspberry-pi-to-arduino-serial-communication-via-usb
ser = serial.Serial('/dev/ttyACM0', 9600)
ser.reset_input_buffer()
s = [0]
# global countCalc
countCalc = 0
#Calculate distance to shape
tempFL = 645.3333333333334
pingsPerRot = 3410


###
    # Matrix node class
###
    class QItem:
        def __int__(self, row, col, dist):
            self.row = row
            self.col = col
            self.dist = dist
        def __repr__(self):
            return f"QItem({self.row}, {self.col}, {self.dist})"

arrayHoogte = []
arrayBasis = []
currentPosRob = [0, 0]
shortestRout = []
neighbours = []
closestNeighbour = [1,1]
weight = 0

testHeight = [1,2,3,4,5,6]
testBase = [1,2,3,4,5,6]
testStart = QItem(3,2,0)
testDest = QItem(4,4,0)




### 
    # Het centrum van de gevonden shapes vinden.
###
def getCenterContour(frame):
    #https://www.geeksforgeeks.org/python-opencv-find-center-of-contour/
    #https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
    M = cv2.moments(cnt)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
        cv2.putText(frame, "center", (cx - 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        return (cx, cy)
        

###
    # Berekenen afstand centrum gevonden shape en middelpunt camerabeeld.
###
def distanceCalculate(p1, p2):
    #p1 and p2 in format (x1,y1) and (x2,y2) tuples
    #p1 is the center of the entire frame, p2 is the center of the contour
#     dis = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    dis = ((p2[0] - p1[0]) ** 2)
    #print("X frame: " + str(p1[0]) + "X shape: " + str(p2[0]))
    return dis

###
    # Signaal naar Robomow sturen om naar rechts te draaien.
###
def turnRight():
    ser.write(b'r')
#     print("do R")

###
    # Signaal naar Robomow sturen om naar rechts te draaien.
###
def turnLeft():
    ser.write(b'l')
#     print("do L")

###
    # Signaal naar Robomow sturen om de omtrek te beginnen bereken.
###
def calcCirc():
    global countCalc
#     while countCalc < 3:
    ser.write(b't')
    countCalc += 1
#     print(countCalc)

###
    # Berekenen Focal Length van mijn camera. Met dit kan ik de afstand tussen de shapes en Robomow berekenen
###
def calcFocalLength():
    knownDis = 44
    known_width = 7.5
    digiWidthInPixels = 110 
    
    focL = (digiWidthInPixels * knownDis)/known_width
    print(focL)

###
    # Berekenen afstand tussen shape en Robomow.
###
def calcDistanceToCam(widthInPixels):
    return (12*tempFL)/widthInPixels

###
    # Maken matrix van oppervlakte
###
def makeMatrixEven(ar1, ar2):
    if(len(ar1) > len(ar2)):
        for i in range(0, len(ar1) - len(ar2)):
            ar2.append("*")
    if(len(ar2) > len(ar1)):
        for i in range(0, len(ar2) - len(ar1)):
            ar1.append("*")
    weight = len(ar1)

###
    # Berekenen gewicht tussen matrix punten
###
def calcWeight(p1, p2):
    dis = abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
#     print(dis)
    return dis
# calcWeight([3,2], [2,1])

def getNeighbours(p):
    if(len(testBase) > p[0]+1):
        nR = [p[0]+1, p[1]]
        neighbours.append(nR)
    if(p[0] > 0):
        nL = [p[0]-1, p[1]]
        neighbours.append(nL)
    if(len(testHeight) > p[1]+1):
        nU = [p[0], p[1]+1]
        neighbours.append(nU)
    if(p[1] > 0):
        nD = [p[0], p[1]-1]
        neighbours.append(nD)
    getClosestNeighbour()
#     print(neighbours)

def getClosestNeighbour():
    global closestNeighbour
    global nextPos
    for n in neighbours:
        if(calcWeight(nextPos, n) < weight):
            closestNeighbour = n
        print(n)    
        nextPos = closestNeighbour
        shortestRout.append(closestNeighbour)
#     print(shortestRout)


# getNeighbours([6,0])
nextPos = testStart
def myDijkstra():
    while(nextPos[0] != testDest[1] and nextPos[1] != testDest[1]):
        getNeighbours(nextPos)

myDijkstra()

###
    # Beeldherkenning gedeelte.
###
def imageRec():
    #https://pysource.com/2019/02/15/detecting-colors-hsv-color-space-opencv-with-python/
    cap = cv2.VideoCapture(0)

    while True:
        
        # Read arduino reply
        if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(line)
                if(line == "doneCirc"):
                    countCalc = 0
                if("pings1: " in line):
                    pings = line[8:]
                    cels1 = math.floor(int(pings)/(pingsPerRot/4))
    #                 print(cels1)
                    for i in range(0, cels1 + 1):
                        arrayHoogte.append("H"+ str(i))
                    currentPosRob[1] = arrayHoogte[len(arrayHoogte) -1]
                    
                if("pings2: " in line):
                    pings = line[8:]
                    cels2 = math.floor(int(pings)/(pingsPerRot/4))
                    for i in range(0, cels2 + 1):
                        arrayBasis.append("B"+ str(i))
                    currentPosRob[0] = arrayBasis[len(arrayBasis) - 1]
                    makeMatrixEven(arrayHoogte, arrayBasis)
                    print(arrayBasis)
                    print(arrayHoogte)
    #                 cap.release()
        
        _, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #https://www.geeksforgeeks.org/webcam-qr-code-scanner-using-opencv/
        data, bbox, _ = detector.detectAndDecode(frame)
        
        if data == "https://www.linkedin.com/in/thierry-klougbo-880b071b1/":
            if(countCalc == 0):
                calcCirc()
                countCalc =+1
        
    #     if(countCalc < 3):
    #         calcCirc()
      
            
        
        # Blue color
        low_blue = np.array([94, 80, 2])
        high_blue = np.array([126, 255, 255])
        blurred = cv2.GaussianBlur(hsv_frame, (7,7), 0)
        blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
        result_blue = cv2.bitwise_and(frame, frame, mask=blue_mask)
        
        #Center of entire frame
        #https://stackoverflow.com/questions/53029540/find-centroid-coordinate-of-whole-frame-in-opencv
        (h, w) = result_blue.shape[:2] #w:image-width and h:image-height
        cv2.circle(result_blue, (w//2, h//2), 7, (255, 255, 255), -1) 
        
        #Shape recognition
        #https://pysource.com/2018/12/29/real-time-shape-detection-opencv-with-python-3/
        #https://pysource.com/2018/03/01/find-and-draw-contours-opencv-3-4-with-python-3-tutorial-19/
        kernel = np.ones((5, 5), np.uint8)
        mask =  cv2.erode(blue_mask, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            
            if area > 400:
                cv2.drawContours(result_blue, [approx], 0, (0, 0, 0), 5)
                if len(approx) == 3:
                    cv2.putText(result_blue, "Triangle", (x, y), 3, 1, (255, 255, 255))
                    if(countCalc == 0):
                        #calc distance to travel
                        
                        # second method calculate distance to travel
                        ## get contour sizes
                        xt,yt,wt,ht = cv2.boundingRect(cnt)
    #                     cv2.putText(result_blue, str(wt), (xt,yt), 3, 1, (255, 255, 255))
                        ndis = calcDistanceToCam(wt)
                        print("str dis: "+ str(ndis))
    #                     calcDisToShape(cnt)
                        
                        #calc distance to turn
                        centerShape = getCenterContour(result_blue)
                        turnDis = distanceCalculate((w//2, h//2), centerShape)
                        print("turndis: " + str(turnDis))
                        if(ndis > 50):
                            if(turnDis >= 51):
                                if(centerShape[0] >= 340):
                                    turnRight()
                                    # Wait for 300milliseconds
                                    #time.sleep(0.500)
                                if(centerShape[0] < 340):
                                    turnLeft()
                                    #time.sleep(0.500)
                            elif(turnDis < 25):
                                if(ndis > 10):
                                    ser.write(b'2')
                                elif(ndis <= 10):
                                    ser.write(b'6')
                        elif(ndis <= 50):
                            if(turnDis >= 225):
                                if(centerShape[0] >= 353):
                                    turnRight()
                                    # Wait for 300milliseconds
                                    #time.sleep(0.500)
                                if(centerShape[0] < 353):
                                    turnLeft()
                                    #time.sleep(0.500)
                            elif(turnDis < 225):
                                if(ndis > 25):
                                    ser.write(b'2')
                                elif(ndis <= 25):
                                    ser.write(b'6')

                                
                        
                elif len(approx) == 4:
                    cv2.putText(result_blue, "Rectangle", (x, y), 3, 1, (255, 255, 255))
    #                 xt,yt,wt,ht = cv2.boundingRect(cnt)
    #                 print(str(wt))
    #                 cv2.putText(result_blue, str(wt), (x,y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
    #                 cv2.rectangle(result_blue, (xt, yt), (xt + wt, yt + ht), (36,255,12), 1)
    #                 calcFocalLength()
                    
    #                 cv2.putText(result_blue, str(wt), (xt,yt), 3, 1, (255, 255, 255))
    #                 if(countCalc == 0):
    #                     calcCirc(countCalc)
    #                     countCalc =+1
    #                 #centerShape = getCenterContour(result_blue)
    #             elif 10 < len(approx) < 20:
    #                 cv2.putText(result_blue, "Circle", (x, y), 3, 1, (255, 255, 255))
        

        #Show results
        cv2.imshow("Frame", frame)    
        #cv2.imshow("Blue", result_blue)
        
        key = cv2.waitKey(1)
        if key == 27:
            break

# if(countCalc == 0):
#             calcCirc()
#             countCalc =+1

