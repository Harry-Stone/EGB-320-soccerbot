# -*- coding: utf-8 -*-
"""
Created on Fri Sep 13 15:32:15 2019

@author: User
"""
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time


fovsize = 1 # 1 radian
fovsamples = 60 #samples in FOV 
field = range(1,fovsamples)
grad = 1/30
obswid = (18+20)/100 #cm

cap = cv2.VideoCapture(0)  		
cap.set(3, 320)                     	# Set the width to 320
cap.set(4, 240)                     	# Set the height to 240
#cap.rotation = 180

# =============================================================================
# Drive Setup
# 
# =============================================================================

# Enable Motors
# Motor A
in1 = 12
in2 = 18
# Motor B
in3 = 13
in4 = 19

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Set all the motor control pins to outputs
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)

PWM0 = GPIO.PWM(in1,1000)
PWM1 = GPIO.PWM(in2,1000)
PWM2 = GPIO.PWM(in3,1000)
PWM3 = GPIO.PWM(in4,1000) 

PWM0.start(0)
PWM1.start(0)
PWM2.start(0)
PWM3.start(0)

def motorspeed0(PWM):
    PWM0.ChangeDutyCycle(int(PWM*0.8))
    return PWM0

def motorspeed1(PWM):
    PWM1.ChangeDutyCycle(int(PWM*0.8))
    return PWM1

def motorspeed2(PWM):
    PWM2.ChangeDutyCycle(int(PWM*0.8))
    return PWM2

def motorspeed3(PWM):
    PWM3.ChangeDutyCycle(int(PWM*0.8))
    return PWM3
 
W = 12.8310 #cm

# =============================================================================
# Drive Setup
# 
# =============================================================================

#KickerSetup
kickpin = 7
GPIO.setup(kickpin, GPIO.OUT)
GPIO.output(kickpin, GPIO.LOW)
GPIO.setup(1, GPIO.IN)


##BALL
def detect():
    A = None
    d = None
    #high_orange = (20, 255, 255)
    #low_orange = (0, 120, 120)
    #high_orange = (16, 230, 255)
    #low_orange = (0, 0, 0)
    high_orange = (17, 255, 255)
    low_orange = (0, 59, 112)

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
            d= (2*360)/r
            A = (x / (320/108)) - 54
		
            if r < r_min:
                mid = None

    # Draw a circle around the ball
    if mid != None:
        #pass
        cv2.circle(frame, mid, int(round(r)), (0,255,0))
        #cv2.putText(frame, str(d), mid, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
        #cv2.putText(frame, str(A), (4,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
    
    cv2.imshow("cam", frame)
    cv2.waitKey(1)
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
    PWM0.start(0)
    PWM1.start(0)
    PWM2.start(0)
    PWM3.start(0)

def Disable_Motor():
    PWM0.stop()
    PWM1.stop()
    PWM2.stop()
    PWM3.stop()

def SetTargetVelocities(fdval,angval):
    left_vel = int((fdval*100) - ((((angval/2)*(180/3.14))*W)/32))
    right_vel = int((fdval*100) + ((((angval/2)*(180/3.14))*W)/32) + 0.5)

    if left_vel > 100:
        motorspeed0(100)
        motorspeed1(0)
    
    elif left_vel > 0:
        motorspeed0(left_vel)
        motorspeed1(0)
    
    elif left_vel < 0:
        motorspeed0(0)
        motorspeed1(abs(left_vel))

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

#def goodbye():
#GPIO.cleanup()
    
# =============================================================================
# Motor Stuff
# =============================================================================
        
def clean():
    GPIO.cleanup()
    
def setdrive(dist, rps):

    #soccerBotSim.SetTargetVelocities(dist, 0, rps)
    #if soccerBotSim.BallInDribbler():
    #    soccerBotSim.KickBall(2)
    if abs(rps)<0.2:
        print(rps)
        SetTargetVelocities(0.30,rps) #if field less than -1.1 reverse? also look at traveling in a curve to go around
        #soccerBotSim.SetTargetVelocities(0,0,rps)
    else:
        print(rps)
        SetTargetVelocities(0,rps)

def BallInDribbler():
    if GPIO.input(1):
        return True
    else:
        return False

def KickBall():
    Disable_Motor()
    GPIO.output(kickpin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(kickpin, GPIO.LOW)
    #Enable_Motor()
    clean()
    quit()

def main():
    Data = detect()
    
    #if Data[0] != None:
       
        #print("Angle: " + str(Data[0]) )
        #print("Distance: " + str(Data[1]) + "\n" )
    #else: 
        #print("no ball\n")

    """ ballRB, blueRB, yellowRB, obstaclesRB = soccerBotSim.GetDetectedObjects()
            if ballRB != None:
                ballRange = ballRB[0]
                ballBearing = ballRB[1] """
            
    if BallInDribbler() == False:
        #objectives = findmax(calcfield(obstaclesRB,ballRB))
        #objectives = findmax(calcfield(None,[Data[1]/100,Data[0]*3.14/180]))
        objectives = findmax(calcfield(None,Data))
    else:
        print('looking for the goal')
        #objectives = findmax(calcfield(None,Data))
        #objectives = findmax(calcfield(None,None))
        #if abs((indextorad(objectives[0]))) < 0.05:
            #KickBall()
        motorspeed0(0)
        motorspeed1(0)
        motorspeed2(0)
        motorspeed3(0)
        time.sleep(1)
        motorspeed0(0)
        motorspeed1(20)
        motorspeed2(0)
        motorspeed3(20.5)
        time.sleep(1)
        motorspeed0(119.5)
        motorspeed1(0)
        motorspeed2(120)
        motorspeed3(0)
        time.sleep(0.5)
        objectives=[0,0]
          
    setdrive(0,indextorad(objectives[0]))
       
    
#print('startup') 
try:
    while True:
        main()
        #print('main loop')
except KeyboardInterrupt:
    clean()
    #print('outoftheloop')
    quit()
    
    
    
    
    
    
#)
