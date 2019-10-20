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
# Enable Motors
EN = 23
# Motor A
in3 = 12
in4 = 18
# Motor B
in1 = 13
in2 = 19

# Set all the motor control pins to outputs
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(EN, GPIO.OUT)

GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)

GPIO.output(EN, GPIO.HIGH)
GPIO.output(22, GPIO.HIGH)

PWM0 = GPIO.PWM(in1,1000)
PWM1 = GPIO.PWM(in2,1000)
PWM2 = GPIO.PWM(in3,1000)
PWM3 = GPIO.PWM(in4,1000)

Drf = GPIO.PWM(17,1000)
Drb = GPIO.PWM(27,1000)

PWM0.start(0)
PWM1.start(0)
PWM2.start(0)
PWM3.start(0)
Drf.start(0)
Drb.start(0)

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
    GPIO.output(EN, GPIO.HIGH)
    PWM0.start(0)
    PWM1.start(0)
    PWM2.start(0)
    PWM3.start(0)

def Disable_Motor():
    GPIO.output(EN, GPIO.LOW)
    PWM0.stop()
    PWM1.stop()
    PWM2.stop()
    PWM3.stop()
    Drf.stop()
    Drb.stop()

def backspeed(fdval,angval):

    W = 12.8310 #cm

    right_vel = int((fdval*100) + ((((angval/2)*(180/3.14))*W)/12))
    left_vel = int((fdval*100) - ((((angval/2)*(180/3.14))*W)/12))

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

       


def SetTargetVelocities(fdval,angval):
    W = 12.8310 #cm

    #backspeed(fdval,angval)

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

def startDribble():
    Drb.ChangeDutyCycle(99)

def stopDribble():
    Drb.ChangeDutyCycle(0)
    Drf.ChangeDutyCycle(0)

def motorkick():
    motorspeed0(0)
    motorspeed1(0)
    motorspeed2(0)
    motorspeed3(0)
    time.sleep(1)
    motorspeed0(0)
    motorspeed1(20)
    motorspeed2(0)
    motorspeed3(20)
    time.sleep(0.5)
    motorspeed0(120)
    motorspeed1(0)
    motorspeed2(120)
    motorspeed3(0)
    Drf.ChangeDutyCycle(90)
    time.sleep(0.5)
    stopDribble()

def clean():
    GPIO.cleanup()
