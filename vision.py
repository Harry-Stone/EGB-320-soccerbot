# -*- coding: utf-8 -*-
"""
Created on Sun Sep 15 21:43:47 2019

@author: pi
"""

import cv2
import numpy as np
import time

def setupVision():
    cap = cv2.VideoCapture(0) 
    cap.set(3, 320)                         # Set the width to 320
    cap.set(4, 240)                         # Set the height to 240
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)                    	# Set the height to 240
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.01)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
    return cap

def takeHsvFrame(cap):
    _, frame = cap.read()
    fil = cv2.GaussianBlur(frame.copy(), (3,3),0)
    return cv2.cvtColor(fil, cv2.COLOR_BGR2HSV)  

def detectBall(hsv_frame,cap):
    A = None
    d = None
    high_orange = (17, 255, 255)
    low_orange = (0, 59, 112)

    r_min = 2 
    _, frame = cap.read()
    img_binary = cv2.inRange(hsv_frame.copy(), low_orange, high_orange)

    img_binary = cv2.dilate(img_binary, None, iterations = 1)

   # Finding Center of Object
    img_contours = img_binary.copy()
    contours = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) [-2]

   # Finding largest contour and locating x,y values and radius
    mid = None 
    r = 0
    if len(contours) > 0:
        circle = max(contours, key = cv2.contourArea)
        ((x,y), r) = cv2.minEnclosingCircle(circle)
        this = cv2.moments(circle)
        if this["m00"] > 0:
            mid = (int(this["m10"] / this["m00"]), int(this["m01"] / this["m00"]))
            ## Focal Length = (radius@10cm * 10cm)/actual radius
            d= (2*360)/r
            A = (x / (320/108)) - 54
		
            if r < r_min:
                mid = None

    return[d,A,mid,r]

def drawBall(vals,frame):
    # Draw a circle around the ball
    mid = vals[2]
    A = vals[1]
    d = vals[0]
    r = vals[3]
    if mid != None:
        #pass
        cv2.circle(frame, mid, int(round(r)), (0,255,0))
        cv2.putText(frame, str(d), mid, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))
        cv2.putText(frame, str(A), (mid[0],(mid[1]+20)), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
        cv2.putText(frame, "Ball", (mid[0],(mid[1]+40)), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))


def detectObstacles(hsv_frame, frame, draw):
    A1 = None
    d1 = None
    x = None
    high_black = np.array([85,70,40])
    low_black = np.array([0,0,0])

    img_binary1 = cv2.inRange(hsv_frame.copy(), low_black, high_black)
    img_binary1 = cv2.dilate(img_binary1, None, iterations = 1)

   # Finding Center 
    img_contours1 = img_binary1.copy()
    contours1 = cv2.findContours(img_contours1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) [-2]

   # Finding largest contour and locating x,y values and width
    obstacle1_d= None
    obstacle1_A= None
    obstacle2_d= None
    obstacle2_A= None
    w_min = 10
    if len(contours1) > 0:
        obstacles = []
        Contour_size = []
        count = 1
        for i, c in enumerate(contours1):
            area = cv2.contourArea(c)
            obstacles.append(area)
        # Sort area by largest to smallest
        Contour_size = sorted(zip(obstacles, contours1), key = lambda x: x[0], reverse = True)

        if len(Contour_size) > 2:
            obstacle3 = Contour_size[2][1]
            obstacle3_area = cv2.minAreaRect(obstacle3)
            obstacle3_box = cv2.boxPoints(obstacle3_area)
            obstacle3_box = np.int0(obstacle3_box)
            obstacle3_w = abs(obstacle3_box[2] - obstacle3_box[0])
            obstacle3_x, obstacle3_y = abs(obstacle3_box[0])
            obstacle3_x2, obstacle3_y2 = obstacle3_w
            if draw == True:
                if obstacle3_x2 >=w_min:
                    drawobstacle3 = cv2.drawContours(frame, [obstacle3_box], 0, (0,255,0),2)
                    obstacle3_d = (15*300)/obstacle3_x2
                    obstacle3_A = (((obstacle3_x + (obstacle3_x2/2)) / (320/108)) - 54)
                    cv2.putText(frame, "D: " + str(obstacle3_d), (4,140), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
                    cv2.putText(frame, "A: " + str(obstacle3_A), (4,125), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
                    cv2.putText(frame, "Obstacle3", (4,110), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))

        if len(Contour_size) > 1:
            obstacle2 = Contour_size[1][1]
            obstacle2_area = cv2.minAreaRect(obstacle2)
            obstacle2_box = cv2.boxPoints(obstacle2_area)
            obstacle2_box = np.int0(obstacle2_box)
            obstacle2_w = abs(obstacle2_box[2] - obstacle2_box[0])
            obstacle2_x, obstacle2_y = abs(obstacle2_box[0])
            obstacle2_x2, obstacle2_y2 = obstacle2_w
            if draw ==True:
                if obstacle2_x2 >=w_min:
                    drawobstacle2 = cv2.drawContours(frame, [obstacle2_box], 0, (0,255,0),2)
                    obstacle2_d = (15*300)/obstacle2_x2
                    obstacle2_A = (((obstacle2_x + (obstacle2_x2/2)) / (320/108)) - 54)
                    cv2.putText(frame, "D: " + str(obstacle2_d), (4,95), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
                    cv2.putText(frame, "A: " + str(obstacle2_A), (4,80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
                    cv2.putText(frame, "Obstacle2", (4,65), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))

       
       
        square = max(contours1, key = cv2.contourArea)
        area = cv2.minAreaRect(square)
        box = cv2.boxPoints(area)
        box = np.int0(box)
        w = abs(box[2] - box[0])
        xyes, yyes = abs(box[0])
        x,y = w
        if draw == True:
            if x >= w_min:
               draw = cv2.drawContours(frame, [box], 0, (0,255,0),2)
               #Focal Length = (width@10cm * 10cm)/actual width = 255
               obstacle1_d = (15*300)/x
               obstacle1_A = (((xyes + (x/2)) / (320/108)) - 54)
               cv2.putText(frame, "D: " + str(obstacle1_d), (4,50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
               cv2.putText(frame, "A: " + str(obstacle1_A), (4,35), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
               cv2.putText(frame, "Obstacle", (4,20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))

    return[[obstacle1_d,obstacle1_A],[obstacle2_d,obstacle2_A]]

## WHITE WALLS

##def detectWalls(hsv_frame, frame)
##
##        low_white = np.array([0,0,0], dtype = np.uint8)
##        high_white = np.array([0 ,0, 255], dtype = np.uint8)
##        white_mask = cv2.inRange(hsv_frame, low_white, high_white)
##        img_binary_white = cv2.bitwise_and(frame, frame, mask = white_mask)
##
##        img_contours_yellow = img_binary_yellow.copy()
##        contours_yellow = cv2.findContours(img_contours_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)



def showCam(frame):
    cv2.imshow("cam", frame)
    key = cv2.waitKey(1)

""" cap = setupVision()

while True:

    start_time = time.time()
    hsv_frame = takeHsvFrame()
    frame = hsv_frame
    ballVals=detectBall(hsv_frame)
    drawBall(ballVals)
    obsVals=detectObstacles(hsv_frame)
    print(obsVals)


#### BLUE GOAL
##
##    low_blue = np.array([94, 80, 2])
##    high_blue = np.array([126, 255, 255])
##    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
##    img_binary2 = cv2.dilate(blue_mask, None, iterations = 1)
##
##    # Finding Center of Object
##    img_contours2 = img_binary2.copy()
##    contours2 = cv2.findContours(img_contours2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) [-2]
##
##
##    # Finding largest contour and locating x,y values and radius
##    mid2 = None 
##    r2 = 0
##    if len(contours2) > 0:
##        yes = max(contours2, key = cv2.contourArea)
##        ((x2,y2), r2) = cv2.minEnclosingCircle(yes)
##        this2 = cv2.moments(yes)
##        if this2["m00"] > 0:
##            mid2 = (int(this2["m10"] / this2["m00"]), int(this2["m01"] / this2["m00"]))
##            ## Focal Length = (radius@10cm * 10cm)/actual radius = 255
##        #    d= (2*225)/r
##            ## Each pixel is (9/80) of an angle
##         #   A = (x / (320/108)) - 54
##        
##            if r < r_min:
##                mid2 = None
##
##    if mid2 != None:
##        cv2.putText(frame, "Blue Goal", mid2, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))



## YELLOW GOAL

    low_yellow = np.array([20,100,100])
    high_yellow = np.array([25,255,255])
    yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)
    img_binary_yellow = cv2.dilate(yellow_mask, None, iterations = 1)

    # Finding Center of Object
    img_contours_yellow = img_binary_yellow.copy()
    contours_yellow = cv2.findContours(img_contours_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) [-2]


    # Finding largest contour and locating x,y values and radius
    mid_yellow = None 
    r_yellow = 0
    r_min = 0
    if len(contours_yellow) > 0:
        square_yellow = max(contours_yellow, key = cv2.contourArea)
        ((x_yellow,y_yellow), r_yellow) = cv2.minEnclosingCircle(square_yellow)
        this2 = cv2.moments(square_yellow)
        if this2["m00"] > 0:
            mid_yellow = (int(this2["m10"] / this2["m00"]), int(this2["m01"] / this2["m00"]))
            ## Focal Length = (radius@10cm * 10cm)/actual radius = 255
            yellowgoal_d = (28*255)/r_yellow
            ## Each pixel is (9/80) of an angle
            yellowgoal_A = (x_yellow / (320/108)) - 54
            print(r_yellow)
        
            if r_yellow < r_min:
                mid_yellow = None
                
    if mid_yellow != None:
        cv2.circle(frame, mid_yellow, int(round(r_yellow)), (0,255,255))
        cv2.putText(frame, "Yellow Goal", mid_yellow, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
        cv2.putText(frame, str(yellowgoal_d), (4, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
        cv2.putText(frame, str(yellowgoal_A), (4,35), cv2.FONT_HERSHEY_PLAIN,1, (0,255,255))




## WHITE WALLS

        low_white = np.array([0,0,0], dtype = np.uint8)
        high_white = np.array([0 ,0, 255], dtype = np.uint8)
        white_mask = cv2.inRange(hsv_frame, low_white, high_white)
        img_binary_white = cv2.bitwise_and(frame, frame, mask = white_mask)

        img_contours_yellow = img_binary_yellow.copy()
        contours_yellow = cv2.findContours(img_contours_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) [-2]

        
        

    
    cv2.imshow("cam", frame)
       
    
    key = cv2.waitKey(1) 
    if key == 27:
        break

    print("Hz: " + str(round(1/(time.time() - start_time))))
 """
