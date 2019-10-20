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
import vision_sunny20
import math
import matplotlib.pyplot as plt
import threading

kernelSanders = np.ones((5,5), np.uint8) ##
fovsize = 1 # 1 radian
fovsamples = 60 #samples in FOV 
field = range(1,fovsamples)
grad = 15/30
obswid = (20+20)/100 #cm
obsMem = [0,0] #number that decrements and is set to something when an obstacle is in the exremity
obsMemFrames = 100 # number of frmaes untill an obstacle is forgotten
scoreGoal = 'blue'
lookingFor = 'ball'
prevLook = 'ball'
max_fd_vel = 0.2#0.2
max_rvel = 0.5
k = 0.8
circleCountdownRunning = False
timeToTurnCircle = 5 #seconds
obsFlag = False

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #goal switch
cap = vision_sunny20.setupVision()
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

GPIO.setup(1, GPIO.IN) #set laser pin as input

def LedBlue():
    GPIO.output(20, GPIO.HIGH)
    GPIO.output(16, GPIO.LOW)

def LedYellow():
    GPIO.output(16, GPIO.HIGH)
    GPIO.output(20, GPIO.LOW)

def ToggleLed():
    global scoreGoal
    if scoreGoal == 'yellow':
        scoreGoal = 'blue'
        LedBlue()
    else:
        scoreGoal = 'yellow'
        LedYellow()
    print('target to score changed to:'+str(scoreGoal))

def checkLedSwitch():
    if GPIO.input(21):
       ToggleLed()
       time.sleep(1)

def convertBallResult(ball):
    if ball[0] != None and ball[1] != None:
        res = [ball[0]/100, ball[1]*3.1415/180]
    else:
        res = None

    return (res)
    

def convertObsResult(obs):
    if obs != None:
        if obs [0] != None:
            for ob in range(len(obs)):
                if obs[ob][0] != None:
                    obs[ob][0]/=100
                    obs[ob][1]*=(3.1415/180)
                    return obs
    return None

def convertGoalResult(goal):
    temphold = convertBallResult(goal)
    if temphold != None:
        temphold = [temphold[1],temphold[0]]
    return temphold

def zeroes (num):
    out = [0] * num
    return out

def changeLook():
    global lookingFor
    global prevLook
    prevLook = lookingFor
    if  lookingFor == 'ball':
        lookingFor = scoreGoal
    elif lookingFor == 'blue':
        lookingFor = 'yellow'
    elif lookingFor == 'yellow':
        lookingFor = 'blue'

    print('looking for: '+lookingFor)
    
def calcRepel(obstacles,field):
    global obsMem
    global obsFlag #
    checkHold = False #
    obsMem[0]-=1
    obsMem[1]-=1
    if (obsMem[0]>0 or obsMem[1]>0) and obstacles == None:
        obstacles = []
    if obsMem[0]>0:
        obstacles.append([0.33,-.39])
    if obsMem[1]>0:
        obstacles.append([0.33,.39])
    if obstacles != None:
        for obstacle in obstacles:
            if obstacle != None:
                if obstacle[0] != None:
                    if obstacle[0]<0.3:
                        checkHold = True #
                        if obstacle[1]>0.4:
                            obsMem[1]=obsMemFrames
                        elif obstacle[1]<-0.4:
                            obsMem[0]=obsMemFrames
                            
                    obstacledist=obstacle[0]
                    obstacleangle=obstacle[1]
                        
                    for i in range(0,fovsamples):
                        
                        obs_falloff = math.atan(0.5*obswid/obstacledist)
                        samplerad = (((-1*fovsamples/2)+i)*(fovsize/fovsamples))
                        if field[i] != -1:
                            field[i]+=(abs(((-1*fovsamples/2)+i)*fovsize/fovsamples-obstacleangle)*0.3)-0.2#(grad/obstacledist)
                        if (samplerad > obstacleangle-obs_falloff and samplerad < obstacleangle+obs_falloff):
                            field[i]=-1
    obsFlag = checkHold #
    return field

    
def calcfield(obstacles, objectiveRB):
    field = zeroes(fovsamples)
    field = calcRepel(obstacles,field)
    if objectiveRB != None:       
        for i in range(0,fovsamples):
            field[i]+=1-abs(((-1*fovsamples/2)+i)*fovsize/fovsamples-objectiveRB[1])*(0.5)
    else:
        field[0]=1
    #visualise(field)
    return field


def visualise(data):
    plt.clf()
    plt.plot(data)
    plt.draw()
    plt.pause(0.001)

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
    if obsFlag:
        JUST_DRIVE_SYSTEM.SetTargetVelocities(-.4,0)
        #print('speed:'+str(max_fd_vel*(1-k*abs(rps/max_rvel)))+' rps:'+str(rps))
        #print('backwards')
    else:
        JUST_DRIVE_SYSTEM.SetTargetVelocities(max_fd_vel*(1-k*abs(rps/max_rvel)),rps)

def BallInDribbler():
    if GPIO.input(1):
        return True #DEAR GOD CHANGE THIS TO True IF YOU NEED IT

    else:
        return False

def main():
    global lookingFor
    global circleCountdownRunning
    global changeLookTimer
    
    hsv_frame = vision_sunny20.takeHsvFrame(cap)
    frame = hsv_frame
    ballRB = vision_sunny20.detectBall(hsv_frame,cap)
    obstaclesRB = convertObsResult(vision_sunny20.detectObstacles(hsv_frame,frame,True, kernelSanders))
    vision_sunny20.drawBall(ballRB,frame)

    if lookingFor == 'yellow':
        objectiveRB = convertGoalResult(vision_sunny20.detectYellowGoal(hsv_frame,frame,cap, kernelSanders))
    elif lookingFor == 'blue':
        objectiveRB = convertGoalResult(vision_sunny20.detectBlueGoal(hsv_frame,frame,cap, kernelSanders))
    else:
        objectiveRB = convertBallResult(ballRB)



    if objectiveRB != None:
        if (lookingFor == 'blue' or lookingFor == 'yellow') and objectiveRB [0] < 0.8:
            changeLook()
            print('close enough to goal for search')

    if BallInDribbler():
        lookingFor = scoreGoal
        JUST_DRIVE_SYSTEM.startDribble()
    elif ballRB != None:
        JUST_DRIVE_SYSTEM.stopDribble()
        if ballRB[0]!=None:
            if circleCountdownRunning:
                changeLookTimer.cancel()
                circleCountdownRunning = False
                print('canceled search countdown because seen ball')
            lookingFor = 'ball'
    else:
        JUST_DRIVE_SYSTEM.stopDribble()


    if circleCountdownRunning == False and objectiveRB == None:
        changeLookTimer = threading.Timer(timeToTurnCircle,changeLook)
        changeLookTimer.start()
        circleCountdownRunning = True
        print('Started time till search')

    potentialField = calcfield(obstaclesRB,objectiveRB)
    decisions = findmax(potentialField)

    if objectiveRB != None:
        if circleCountdownRunning:
            changeLookTimer.cancel()
            circleCountdownRunning = False
            print('canceled search countdown' + str(objectiveRB))

        
        #print(objectiveRB[0])
        if BallInDribbler() and lookingFor == scoreGoal and objectiveRB[0] < 0.8 and abs(indextorad(decisions[0])) < 0.05 and objectiveRB != ballRB:
            JUST_DRIVE_SYSTEM.motorkick()
            lookingFor = 'ball'

    setdrive(decisions[1],indextorad(decisions[0]))
    vision_sunny20.showCam(frame)
    checkLedSwitch()
  
try:
    plt.ion()
    plt.show()
    ToggleLed()
    while True:
        main()
except KeyboardInterrupt:
    clean()
    quit()
            
