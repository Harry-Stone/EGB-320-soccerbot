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
import matplotlib.pyplot as plt
import sys

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

fovsize = 1 # 1 radian
fovsamples = 60 #samples in FOV 
field = range(1,fovsamples)
grad = 1/30
obswid = (18+20)/100 #cm
obsMem = [0,0] #number that decrements and is set to something when an obstacle is in the exremity
obsMemFrames = 15 # number of frmaes untill an obstacle is forgotten

cap = vision.setupVision()

#KickerSetup
kickpin = 7
GPIO.setup(kickpin, GPIO.OUT)
GPIO.output(kickpin, GPIO.LOW)
GPIO.setup(1, GPIO.IN)

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

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

    
def calcfield(obstacles, ballRB):
    field = zeroes(fovsamples)
    field = calcRepel(obstacles,field)
    if ballRB != None:
        if ballRB[0]!=None:
            for i in range(0,fovsamples):
                field[i]+=1-abs(((-1*fovsamples/2)+i)*fovsize/fovsamples-ballRB[1])*(0.5)
        #visualise(field)
        return field


def visualise(data):
    plt.clf()
    plt.plot(data)
    plt.draw()
    plt.pause(0.001)

def findmax(array):
    bestindex =-1
    bestval = -100
    midval = array[30]
    for i in range(len(array)):
        #print(str(array[i])+' vs '+str(bestval))
        if array[i]>bestval:
            bestval=array[i]
            bestindex=i
            #print(bestindex)
    #print(array)
    #print('midval:'+str(midval)+'    Best Index:'+str(bestindex))
    return[bestindex,midval]


def indextorad(index):
    return (((-1*fovsamples/2)+index)*(fovsize/fovsamples))


def clean():
    GPIO.cleanup()


def setdrive(dist, rps):
        
    JUST_DRIVE_SYSTEM.SetTargetVelocities(dist,rps)
    #print('Speed forward:'+ str(dist)+'       Turn Amount:'+str(rps))
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

def keyDrive(input):
    if input == 'w':
        JUST_DRIVE_SYSTEM.SetTargetVelocities(0.2,0)
    elif input == 'W':
        JUST_DRIVE_SYSTEM.SetTargetVelocities(1,0)
    elif input == 's':
        print('s')
        JUST_DRIVE_SYSTEM.SetTargetVelocities(-0.2,0)
    elif input == 'S':
        JUST_DRIVE_SYSTEM.SetTargetVelocities(-1,0)
    elif input == 'd':
        JUST_DRIVE_SYSTEM.SetTargetVelocities(0,0.2)
    elif input == 'D':
        JUST_DRIVE_SYSTEM.SetTargetVelocities(0,0.5)
    elif input == 'a':
        JUST_DRIVE_SYSTEM.SetTargetVelocities(0,-0.2)
    elif input == 'A':
        JUST_DRIVE_SYSTEM.SetTargetVelocities(0,-0.5)
    elif input == 'K' or input == 'k':
        JUST_DRIVE_SYSTEM.motorkick()
    else:
        JUST_DRIVE_SYSTEM.SetTargetVelocities(0,0)


def main():

    hsv_frame = vision.takeHsvFrame(cap)
    frame = hsv_frame
    ballVals= vision.detectBall(hsv_frame,cap)
    obsVals= vision.detectObstacles(hsv_frame,frame,True)
    vision.drawBall(ballVals,frame)
    vision.showCam(frame)
    key = sys.stdin.read(1)
    print(key)
    keyDrive(key)
try:
    #plt.ion()
    #plt.show()
    while True:
        main()
except KeyboardInterrupt:
    clean()
    quit()
