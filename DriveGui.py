# -*- coding: utf-8 -*-
"""
Created on Mon Sep  9 17:37:47 2019

@author: User
"""

import RPi.GPIO as GPIO
from time import sleep

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

PWM0 = GPIO.PWM(in1,100)
PWM1 = GPIO.PWM(in2,100)
PWM2 = GPIO.PWM(in3,100)
PWM3 = GPIO.PWM(in4,100) 

def motorspeed0(PWM):
    PWM0.ChangeDutyCycle(PWM*0.8)
    return PWM0

def motorspeed1(PWM):
    PWM1.ChangeDutyCycle(PWM*0.8)
    return PWM1

def motorspeed2(PWM):
    PWM2.ChangeDutyCycle(PWM*0.8)
    return PWM2

def motorspeed3(PWM):
    PWM3.ChangeDutyCycle(PWM*0.8)
    return PWM3
 
W = 12.8310 #cm

# Setup same as test code
def Enable_Motor_Drive():
    PWM0.start(0)
    PWM1.start(0)
    PWM2.start(0)
    PWM3.start(0)

def Disable_Motor_Drive():
    PWM0.stop()
    PWM1.stop()
    PWM2.stop()
    PWM3.stop()

def Top_Speed_Drive():
    motorspeed0(80)
    motorspeed1(0)
    motorspeed2(80)
    motorspeed3(0)
    sleep(2)
    motorspeed0(0)
    motorspeed2(0)

def Slow_Speed_Drive():
    motorspeed0(12)
    motorspeed1(0)
    motorspeed2(12)
    motorspeed3(0)
    sleep(17)
    motorspeed0(0)
    motorspeed2(0)
    
def Forward_Angle_Speed_Drive(forward, angle):
       
    left_vel = round(forward - ((angle*W)/2),1)
    right_vel = round(forward + ((angle*W)/2),1)

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
        
    sleep(3)
    
    motorspeed0(0)
    motorspeed1(0)
    motorspeed2(0)
    motorspeed3(0)
