# -*- coding: utf-8 -*-
"""
Created on Fri Sep 13 15:32:15 2019

@author: User
"""
import time
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# =============================================================================
# Drive Setup
# 
# =============================================================================
#def  Motor_Setup():
# Enable Motors
# Motor A
in1 = 12
in2 = 18
# Motor B
in3 = 13
in4 = 19

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
#print("motorsetup")

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

# =============================================================================
# Drive Setup
# 
# =============================================================================


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
    W = 12.8310 #cm

    if angval > 0.4:
        angval = angval
    elif angval > 0.3:
        angval = angval*1.1
    elif angval > 0.2:
        angval = angval*1.2
    elif angval > 0.1:
        angval = angval*1.3
    elif angval > 0.05 or angval < 0.05:
        angval = angval*1.4

    #print(str(fdval))
    #if fdval > 90:
    #    fdval = 0.9
    #elif fdval > 80:
    #    fdval = 0.8
    #elif fdval > 70:
    #    fdval = 0.7
    #elif fdval > 60:
    #    fdval = 0.6
    #elif fdval > 50:
    #    fdval = 0.5
    #elif fdval > 40:
    #    fdval = 0.4
    #elif fdval > 30:
    #    fdval = 0.3
    #elif fdval > 20:
    #    fdval = 0.2
    #elif fdval > 10:
    #    fdval = 0.1
    #print(str(fdval)+"\n")

    
    right_vel = int((fdval*100) - ((((angval/2)*(180/3.14))*W)/12))
    left_vel = int((fdval*100) + ((((angval/2)*(180/3.14))*W)/12))

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

    
# =============================================================================
# Motor Stuff
# =============================================================================


def motorkick():
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


def clean():
    GPIO.cleanup()
