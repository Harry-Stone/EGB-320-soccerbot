# -*- coding: utf-8 -*-
"""
Created on Fri Sep 13 15:32:15 2019

@author: User
"""
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import JUST_DRIVE_SYSTEM
import vision
import math

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

fovsize = 1 # 1 radian
fovsamples = 60 #samples in FOV 
field = range(1,fovsamples)
grad = 1/30
obswid = (18+20)/100 #cm

cap = vision.setupVision()

#KickerSetup
kickpin = 7
GPIO.setup(kickpin, GPIO.OUT)
GPIO.output(kickpin, GPIO.LOW)
GPIO.setup(1, GPIO.IN)

def convertBallResult(ball):
    if ball[0] != None and ball[1] != None:
        res = [ball[0]/100, ball[1]*3.1415/180]
    else:
        res = ball

    return (res)

def convertObsResult(obs):
    for ob in range(len(obs)):
        if obs[ob][0] != None:
            obs[ob][0]/=100
            obs[ob][1]*=(3.1415/180)


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


def clean():
    GPIO.cleanup()


def setdrive(dist, rps):

    #Speed1 = 0.3

    #soccerBotSim.SetTargetVelocities(dist, 0, rps)
    #if soccerBotSim.BallInDribbler():
    #    soccerBotSim.KickBall(2)
    if abs(rps)<0.2:
        #print(rps)

        if dist > 0.3:
            Speed1 = dist
        elif dist > 0.2:
            Speed1 = 0.25
        elif dist > 0.1:
            Speed1 = 0.2
        else:
            Speed1 = 0.3
        
        JUST_DRIVE_SYSTEM.SetTargetVelocities(Speed1,rps) #if field less than -1.1 reverse? also look at traveling in a curve to go around
        #soccerBotSim.SetTargetVelocities(0,0,rps)
    else:
        #print(rps)
        JUST_DRIVE_SYSTEM.SetTargetVelocities(0,rps)

def BallInDribbler():
    if GPIO.input(1):
        return True
    else:
        return False

def KickBall():
    JUST_DRIVE_SYSTEM.Disable_Motor()
    GPIO.output(kickpin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(kickpin, GPIO.LOW)
    JUST_DRIVE_SYSTEM.Enable_Motor()
    #clean()
    #quit()


def main():#DriveSetup):
	
#    if DriveSetup == 0:
        #JUST_DRIVE_SYSTEM.Motor_Setup()
#        DriveSetup = 1
    hsv_frame = vision.takeHsvFrame(cap)
    frame = hsv_frame
    ballVals= vision.detectBall(hsv_frame,cap)
    vision.drawBall(ballVals,frame)
    obsVals= vision.detectObstacles(hsv_frame,frame)
    #print(obsVals)
    
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
        objectives = findmax(calcfield(convertObsResult(obsVals),convertBallResult(ballVals)))
    else:
        #print('looking for the goal')
        #objectives = findmax(calcfield(None,Data))
        #objectives = findmax(calcfield(None,None))
        #if abs((indextorad(objectives[0]))) < 0.05:
            #KickBall()
        
        #JUST_DRIVE_SYSTEM.motorkick()
        #objectives=[0,0]

        objectives = findmax(calcfield(convertObsResult(obsVals),convertBallResult(ballVals)))
          
    setdrive(0,indextorad(objectives[0]))
    vision.showCam(frame)
    
try:
    while True:
        main()
except KeyboardInterrupt:
    clean()
    quit()
