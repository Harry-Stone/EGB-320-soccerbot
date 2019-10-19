# -*- coding: utf-8 -*-
"""
Created on Mon Apr 29 20:25:30 2019

@author: Scheckie
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Apr 11 13:08:57 2019

@author: Scheckie
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import os


imgNums = [i for i in range(897)]+[i for i in range(1080,1815)]+[i for i in range(2736,3429)] #only the useful images

minVals = np.array([0,0,0])
maxVals = np.array([255,255,255])
buttonPos = 1;
def H_trackbarMin(val):
    global minVals
    minVals[0] = val
def S_trackbarMin(val):
    global minVals
    minVals[1] = val
def V_trackbarMin(val):
    global minVals
    minVals[2] = val
def H_trackbarMax(val):
    global maxVals
    maxVals[0] = val
def S_trackbarMax(val):
    global maxVals
    maxVals[1] = val
def V_trackbarMax(val):
    global maxVals
    maxVals[2] = val
def buttonPress(val):
    global buttonPos
    buttonPos = val

FPS = 10 #How fast to go through the images. IS APPROXIMATE, your code will reduce this.

imgFolder = "C:\\Users\\User\\Desktop\\RunTimeVideo3" # replace with a string to where you saved the images.

cv2.namedWindow("trackBars",cv2.WINDOW_NORMAL)
#cv2.resizeWindow("video",240*2+200,320*2)
cv2.createTrackbar('Hmin','trackBars',0,255,H_trackbarMin)
cv2.createTrackbar('HMax','trackBars',255,255,H_trackbarMax)
cv2.createTrackbar('Smin','trackBars',0,255,S_trackbarMin)
cv2.createTrackbar('Smax','trackBars',255,255,S_trackbarMax)
cv2.createTrackbar('Vmin','trackBars',0,255,V_trackbarMin)
cv2.createTrackbar('Vmax','trackBars',255,255,V_trackbarMax)
cv2.createTrackbar('Binary','trackBars',1,1,buttonPress)
cap = cv2.VideoCapture(0) 
cap.set(3, 320)                        
cap.set(4, 240)

os.system("v4l2-ctl --set-ctrl=white_balance_auto_preset=4")#########
os.system("v4l2-ctl --set-ctrl=auto_exposure=1")
os.system("v4l2-ctl --set-ctrl=exposure_time_absolute=450")

cap.set(cv2.CAP_PROP_EXPOSURE, 0.6)                    	
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.05) 
cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)

kernel = np.ones((5,5), np.uint8)

while True:
    
    _, img = cap.read()
    img = img[0:190, 0:320]
    _, frame = cap.read()
    frame = frame[0:190, 0:320]
    # Your code here 
    hsvImg = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    binaryMask = cv2.inRange(hsvImg,minVals,maxVals)
    
    binaryMask = cv2.erode(binaryMask, kernel, iterations = 0)
    binaryMask = cv2.dilate(binaryMask, kernel, iterations = 0)
    
    
    if buttonPos:
        threshedImg = cv2.bitwise_and(img,img,mask = binaryMask)
    
    
    
    
    
    
    # show the desired images
    cv2.imshow("video",img)
    if buttonPos:
        cv2.imshow("Thresholded Image",threshedImg)
    else:
        cv2.imshow("Thresholded Image",binaryMask)
    
    
    
    
    
    if (cv2.waitKey(int(1000/FPS)) & 0xff == ord('q')): # required to actually display the images, and quit.
        break
    
    
# cleanup
cv2.destroyAllWindows()



