# -*- coding: utf-8 -*-
"""
Created on Mon Sep  9 17:37:47 2019

@author: User
"""

import RPi.GPIO as GPIO
from time import sleep
from tkinter import Tk, Label, Button, Scale

# tkinter GUI basic settings

GUI = Tk()
GUI.title("Drive Demo")
GUI.config(background= "#0080FF")
GUI.minsize(500,300)
GUI.grid_columnconfigure(2, weight = 1)
GUI.grid_rowconfigure(2, weight = 1)


# Enable Motors
EN = 21
# Motor A
in1 = 18
in2 = 12
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
GPIO.setup(EN, GPIO.OUT)

PWM0 = GPIO.PWM(in1,1000)
PWM1 = GPIO.PWM(in2,1000)
PWM2 = GPIO.PWM(in3,1000)
PWM3 = GPIO.PWM(in4,1000) 

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
#forward_vel = 60 #cm
#ang_vel = -2 #cm

# Setup same as test code
def Enable_Motor():
    GPIO.output(EN, GPIO.HIGH)
    PWM0.start(0)
    PWM1.start(0)
    PWM2.start(0)
    PWM3.start(0)
    Text3 = Label(GUI,text='Motor Enabled', fg='green', bg = '#0080FF', padx = 0)
    Text3.grid(row=1,column=1)

def Disable_Motor():
    GPIO.output(EN, GPIO.LOW)
    PWM0.stop()
    PWM1.stop()
    PWM2.stop()
    PWM3.stop()
    Text3 = Label(GUI,text='Motor Disbaled', fg='red', bg = '#0080FF', padx = 0)
    Text3.grid(row=1,column=1)

def Top_Speed():
    Text2 = Label(GUI,text='     Top Speed     ', bg = '#0080FF', fg='white', padx = 0)
    Text2.grid(row=0,column=1)
    motorspeed0(90)
    motorspeed1(0)
    motorspeed2(90)
    motorspeed3(0)
    sleep(10)
    motorspeed0(0)
    motorspeed2(0)

def Slow_Speed():
    Text2 = Label(GUI,text='      Slow Speed     ', bg = '#0080FF', fg='white', padx = 0)
    Text2.grid(row=0,column=1)
    motorspeed0(8)
    motorspeed1(0)
    motorspeed2(8)
    motorspeed3(0)
    sleep(14)
    motorspeed0(0)
    motorspeed2(0)
    
def Forward_Angle_Speed():
    Text2 = Label(GUI,text='    Angle Speed    ', bg = '#0080FF', fg='white', padx = 0)
    Text2.grid(row=0,column=1)
    
        
    left_vel = round(forward_vel.get() - ((ang_vel.get()*W)/2),1)
    right_vel = round(forward_vel.get() + ((ang_vel.get()*W)/2),1)

    if left_vel > 100:
        motorspeed0(100)
        motorspeed1(0)
    
    elif left_vel > 0:
        motorspeed0(left_vel)
        motorspeed1(0)

    elif left_vel < -100:
        motorspeed0(0)
        motorspeed1(100)
    
    elif left_vel < 0:
        motorspeed0(0)
        motorspeed1(abs(left_vel))

    

    else:
        pass
        #motorspeed0(0)
        #motorspeed1(0)

    if right_vel > 100:
        motorspeed2(100)
        motorspeed3(0)
    
    elif right_vel > 0:
        motorspeed2(right_vel)
        motorspeed3(0)

    elif right_vel < -100:
        motorspeed2(0)
        motorspeed3(100)
    
    elif right_vel < 0:
        motorspeed2(0)
        motorspeed3(abs(right_vel))

    else:
        #motorspeed2(0)
        #motorspeed3(0)
        pass
    sleep(3)
    
    motorspeed0(0)
    motorspeed1(0)
    motorspeed2(0)
    motorspeed3(0)
    
def goodbye():
    GPIO.cleanup()
    #quit()
    

Text1 = Label(GUI,text='Drive Test:', fg='#FFFFFF', bg = '#0080FF', padx = 50, pady = 50)
Text1.grid(row=0,column=0)

Text2 = Label(GUI,text='Waiting for Input', fg='#FFFFFF', bg = '#0080FF', padx = 0)
Text2.grid(row=0,column=1)

Text3 = Label(GUI,text='Motor Disbaled', fg='red', bg = '#0080FF', padx = 0)
Text3.grid(row=1,column=1)

Button1 = Button(GUI, text='Top Speed', command = Top_Speed, bg='bisque2', height = 1, width = 12)
Button1.grid(row=2,column=0)

Button2 = Button(GUI, text='Slow Speed', command = Slow_Speed, bg='bisque2', height = 1, width = 12)
Button2.grid(row=2,column=1)

Button3 = Button(GUI, text='Angle Speed', command = Forward_Angle_Speed, bg='bisque2', height = 1, width = 12)
Button3.grid(row=2,column=2)

Button4 = Button(GUI, text='Enable Motor', command = Enable_Motor, bg='bisque2', height = 1, width = 12)
Button4.grid(row=3,column=0)

Button5 = Button(GUI, text='QUIT', command = goodbye, bg='bisque2', height = 1, width = 12)
Button5.grid(row=3,column=1)

Button6 = Button(GUI, text='Disable Motor', command = Disable_Motor, bg='bisque2', height = 1, width = 12)
Button6.grid(row=3,column=2)

forward_vel = Scale(GUI, from_=100, to=-100, resolution = 0.1)
forward_vel.grid(row=1,column=2)

ang_vel = Scale(GUI, from_=5, to = -5, resolution = 0.01)
ang_vel.grid(row=1,column=3)

ang_vel

GUI.mainloop()

GPIO.cleanup()
