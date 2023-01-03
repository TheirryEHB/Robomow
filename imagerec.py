import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

print(cv2.__version__)

#https://pysource.com/2019/02/15/detecting-colors-hsv-color-space-opencv-with-python/
cap = cv2.VideoCapture(0)
while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
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
            elif len(approx) == 4:
                cv2.putText(frame, "Rectangle", (x, y), 3, 1, (0, 0, 0))
            elif 10 < len(approx) < 20:
                cv2.putText(frame, "Circle", (x, y), 3, 1, (0, 0, 0))
        
    
    #Show results
    cv2.imshow("Frame", frame)
    
    #cv2.imshow("Blue", result_blue)
    
    key = cv2.waitKey(1)
    if key == 27:
        break


def nothing(x):
    # any operation
   pass
