import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import serial

print(cv2.__version__)

#https://www.instructables.com/Raspberry-Pi-Arduino-Serial-Communication/
#https://www.arrow.com/en/research-and-events/articles/raspberry-pi-to-arduino-serial-communication-via-usb
ser = serial.Serial('/dev/ttyACM0'm 9600)
s = [0]

def getCenterContour(frame):
    #https://www.geeksforgeeks.org/python-opencv-find-center-of-contour/
    #https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
    M = cv2.moments(cnt)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
        return (cx, cy)
        #cv2.putText(frame, "center", (cx - 20, cy - 20),
         #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
         
def distanceCalculate(p1, p2):
    #p1 and p2 in format (x1,y1) and (x2,y2) tuples
    #p1 is the center of the entire frame, p2 is the center of the contour

    dis = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    #print("X frame: " + str(p1[0]) + "X shape: " + str(p2[0]))
    return dis

def turnRight():
    ser.write(b'0')
    print("do R")
def turnLeft():
    ser.write(b'1')
    print("do L")

#https://pysource.com/2019/02/15/detecting-colors-hsv-color-space-opencv-with-python/
cap = cv2.VideoCapture(0)
while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #Center of entire frame
    #https://stackoverflow.com/questions/53029540/find-centroid-coordinate-of-whole-frame-in-opencv
    (h, w) = frame.shape[:2] #w:image-width and h:image-height
    cv2.circle(frame, (w//2, h//2), 7, (255, 255, 255), -1) 
    
    # Blue color
    low_blue = np.array([94, 80, 2])
    high_blue = np.array([126, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    result_blue = cv2.bitwise_and(frame, frame, mask=blue_mask)
    
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
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)
            if len(approx) == 3:
                cv2.putText(frame, "Triangle", (x, y), 3, 1, (0, 0, 0))
                centerShape = getCenterContour(frame)
                dis = distanceCalculate((w//2, h//2), centerShape)
                if(dis > 2):
                    if(centerShape[0] > 340):
                        turnRight()
                        # Wait for 300milliseconds
                        #time.sleep(0.500)
                    if(centerShape[0] < 300):
                        turnLeft()
                        #time.sleep(0.500)
                    
            elif len(approx) == 4:
                cv2.putText(frame, "Rectangle", (x, y), 3, 1, (0, 0, 0))
                getCenterContour(frame)
            elif 10 < len(approx) < 20:
                cv2.putText(frame, "Circle", (x, y), 3, 1, (0, 0, 0))
        
    #Show results
    cv2.imshow("Frame", frame)    
    #cv2.imshow("Blue", result_blue)
    
    key = cv2.waitKey(1)
    if key == 27:
        break

