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
#import matplotlib.pyplot as plt

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

fovsize = 1 # 1 radian
fovsamples = 60 #samples in FOV 
field = range(1,fovsamples)
grad = 1/30
obswid = (20+20)/100 #cm
obsMem = [0,0] #number that decrements and is set to something when an obstacle is in the exremity
obsMemFrames = 15 # number of frmaes untill an obstacle is forgotten
goal = 'blue'
max_fd_vel = 0.2
max_rvel = 0.5
k = 0.8

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
    return obs


def zeroes (num):
    out = [0] * num
    return out

    
def calcRepel(obstacles,field):
    obsMem[0]-=1
    obsMem[1]-=1
    if (obsMem[0]>0 or obsMem[1]>0) and obstacles == None:
        obstacles = []
    if obsMem[0]>0:
        obstacles.append([0.33,-.43])
    if obsMem[1]>0:
        obstacles.append([0.33,.43])
    if obstacles[len(obstacles)-1][0] != None:
        for obstacle in obstacles:
            if obstacle[0]!= None:
                if obstacle[0]<1.2:
                    if obstacle[0]<0.33:
                        if obstacle[1]>0.44:
                            obsMem[1]=obsMemFrames
                        elif obstacle[1]<-0.44:
                            obsMem[0]=obsMemFrames
            
                    for i in range(0,fovsamples):
                        obstacledist=obstacle[0]
                        obstacleangle=obstacle[1]
                        obs_falloff = math.atan(0.5*obswid/obstacledist)
                        samplerad = (((-1*fovsamples/2)+i)*(fovsize/fovsamples))
                        if field[i] != -1:
                            field[i]+=(abs(((-1*fovsamples/2)+i)*fovsize/fovsamples-obstacleangle)*0.3)-0.2#(grad/obstacledist)
                        if (samplerad > obstacleangle-obs_falloff and samplerad < obstacleangle+obs_falloff):
                            field[i]=-1

    return field

    
def calcfield(obstacles, objectiveRB):
    field = zeroes(fovsamples)
    field = calcRepel(obstacles,field)
    if objectiveRB[1] != None:       
        for i in range(0,fovsamples):
            field[i]+=1-abs(((-1*fovsamples/2)+i)*fovsize/fovsamples-objectiveRB[1])*(0.5)
    else:
        field[0]=1
    #visualise(field)
    return field



def visualise(data):
    pass

def findmax(array):
    bestindex =-1
    bestval = -1
    midval = array[30]
    for i in range(0,len(array)):
        if array[i]>bestval:
            bestval=array[i]
            bestindex=i
    return[bestindex,midval]


def indextorad(index):
    return (((-1*fovsamples/2)+index)*(fovsize/fovsamples))


def clean():
    GPIO.cleanup()


def setdrive(dist, rps):
    JUST_DRIVE_SYSTEM.SetTargetVelocities(max_fd_vel*(1-k*abs(rps/max_rvel)),rps)
    print('speed:'+str(max_fd_vel*(1-k*abs(rps/max_rvel)))+' rps:'+str(rps))


def BallInDribbler():
    if GPIO.input(1):
        return False #DEAR GOD CHANGE THIS TO True IF YOU NEED IT

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
    obsVals= vision.detectObstacles(hsv_frame,frame,True)
    vision.drawBall(ballVals,frame)
    field = calcfield(convertObsResult(obsVals),convertBallResult(ballVals))
    objectives = findmax(field)
          
    setdrive(objectives[1],indextorad(objectives[0]))
    if BallInDribbler():
        JUST_DRIVE_SYSTEM.motorkick()
    #print('objectives'+str(objectives))
    vision.showCam(frame)
    
try:
    #plt.ion()
    #plt.show()
    while True:
        main()
except KeyboardInterrupt:
    clean()
    quit()
