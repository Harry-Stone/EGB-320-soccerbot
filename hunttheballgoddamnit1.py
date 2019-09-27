# -*- coding: utf-8 -*-
"""
Created on Fri Sep 13 15:32:15 2019

@author: User
"""
#import RPi.GPIO as GPIO
import cv2
import numpy as np


fovsize = 1 # 1 radian
fovsamples = 60 #samples in FOV 
field = range(1,fovsamples)
grad = 1/30
obswid = (18+20)/100 #cm

cap = cv2.VideoCapture(0)  		
cap.set(3, 320)                     	# Set the width to 320
cap.set(4, 240)                     	# Set the height to 240


# =============================================================================
# Drive Setup
# 
# =============================================================================

def motorspeed0(PWM):
    #PWM0.ChangeDutyCycle(int(PWM*0.8))
    print(int(PWM*0.8))

def motorspeed1(PWM):
    print(int(PWM*0.8))

def motorspeed2(PWM):
    print(int(PWM*0.8))

def motorspeed3(PWM):
    print(int(PWM*0.8))
 
W = 12.8310 #cm

# =============================================================================
# Drive Setup
# 
# =============================================================================



## Ball 



##BALL
def detect():
    A = None
    d = None
    high_orange = (20, 255, 255)
    low_orange = (0, 120, 120)

    r_min = 2 
    _, frame = cap.read()
    fil = cv2.GaussianBlur(frame.copy(), (3,3),0)
    hsv_frame = cv2.cvtColor(fil, cv2.COLOR_BGR2HSV)
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
            d= (2*225)/r
            A = (x / (320/108)) - 54
		
            if r < r_min:
                mid = None

    # Draw a circle around the ball
    if mid != None:
        pass
        #cv2.circle(frame, mid, int(round(r)), (0,255,0))
        #cv2.putText(frame, str(d), mid, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
        #cv2.putText(frame, str(A), (4,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
    
    #cv2.imshow("cam", frame)
    #cv2.waitKey()
    if d != None and A != None:
        res = [d/100, A*3.1415/180]
    else:
        res = [d,A]

    return (res)

def zeroes (num):
    out = [0] * num
    return out

    
def calcfield(obstacles, ball):
    field = zeroes(fovsamples)
    
    if obstacles != None:
        for obstacle in obstacles:
            for i in range(0,fovsamples):
                obstacledist=obstacle[0]
                obstacleangle=obstacle[1]
                obs_falloff = math.atan(0.5*obswid/obstacledist)
                samplerad = (((-1*fovsamples/2)+i)*(fovsize/fovsamples))
                #field[i]+=abs(((-1*fovsamples/2)+i)*fovsize/fovsamples-obstacleangle)*(grad/obstacledist)
                if samplerad > obstacleangle-obs_falloff and samplerad < obstacleangle+obs_falloff:
                    field[i]=-1/obstacledist
                #need to impliment a falloff for the obstacle
                #may be able to get rid of the for loop

    if ball[0] != None:     
        for i in range(0,fovsamples):
            field[i]+=(1-abs(((-1*fovsamples/2)+i)*fovsize/fovsamples-ball[1])*(1))/ball[0]
        
    field[0]=0.01
    field[fovsamples-1]=0.02 
    visualise(field)
    return field


def visualise(data):
    pass

def findmax(array):
    bestindex =-1
    bestval = -1
    for i in range(0,len(array)):
        if array[i]>bestval:
            bestval=array[i]
            bestindex=i
    return[bestindex,bestval]

def indextorad(index):
    return (((-1*fovsamples/2)+index)*(fovsize/fovsamples))


# =============================================================================
# Motor Stuff
# =============================================================================

# Setup same as test code
def Enable_Motor():
    pass

def Disable_Motor():
    pass

def SetTargetVelocities(fdval,angval):
    #print(angval)
    left_vel = int((fdval*100) - (((angval*(180/3.14))*W)/2))
    right_vel = int((fdval*100) + (((angval*(180/3.14))*W)/2))

    if left_vel > 100:
        motorspeed0(100)
        motorspeed1(0)
    
    elif left_vel > 0:
        motorspeed0(right_vel)
        motorspeed1(0)
    
    elif left_vel < 0:
        motorspeed0(0)
        motorspeed1(abs(right_vel))

    elif left_vel < -100:
        motorspeed0(0)
        motorspeed1(100)

    else:
        motorspeed0(0)
        motorspeed1(0)

    if right_vel > 100:
        motorspeed2(100)
        motorspeed3(0)
    
    elif right_vel > 0:
        motorspeed2(right_vel)
        motorspeed3(0)
    
    elif right_vel < 0:
        motorspeed2(0)
        motorspeed3(abs(right_vel))

    elif right_vel < -100:
        motorspeed2(0)
        motorspeed3(100)

    else:
        motorspeed2(0)
        motorspeed3(0)

def goodbye():
    pass
    
# =============================================================================
# Motor Stuff
# =============================================================================
        
        
    
def setdrive(dist, rps):

    #soccerBotSim.SetTargetVelocities(dist, 0, rps)
    #if soccerBotSim.BallInDribbler():
    #    soccerBotSim.KickBall(2)
    if abs(rps)<0.05:
        SetTargetVelocities(0.3,rps) #if field less than -1.1 reverse? also look at traveling in a curve to go around
        #soccerBotSim.SetTargetVelocities(0,0,rps)
    else:
        SetTargetVelocities(0,rps) #fix the from the left obs stuck loop

def BallInDribbler():
    return False

def KickBall():
    pass

def main():
    Data = detect()
    
    if Data[0] != None:
        pass
        #print("Angle: " + str(Data[0]) )
        #print("Distance: " + str(Data[1]) )
    else: 
        print("no ball")

    """ ballRB, blueRB, yellowRB, obstaclesRB = soccerBotSim.GetDetectedObjects()
            if ballRB != None:
                ballRange = ballRB[0]
                ballBearing = ballRB[1] """
            
    if BallInDribbler() == False:
        #objectives = findmax(calcfield(obstaclesRB,ballRB))
        #objectives = findmax(calcfield(None,[Data[1]/100,Data[0]*3.14/180]))
        objectives = findmax(calcfield(None,Data))
    else:
        objectives = findmax(calcfield(None,None))
        if abs((indextorad(objectives[0]))) < 0.05:
            KickBall()
            
    setdrive(0,indextorad(objectives[0]))
       

def clean():
    GPIO.cleanup()
    
print('startup') 
try:
    while True:
        main()
except KeyboardInterrupt:
    clean()
    
    
    
    
    
    
#)
