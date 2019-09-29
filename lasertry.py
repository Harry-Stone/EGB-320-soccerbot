# -*- coding: utf-8 -*-
"""
Created on Mon Sep  9 17:37:47 2019

@author: User
"""

import RPi.GPIO as GPIO
from tkinter import Tk, Button, Label

# tkinter GUI basic settings

GUI = Tk()
GUI.title("Drive Demo")
GUI.config(background= "#0080FF")
GUI.minsize(500,300)
GUI.grid_columnconfigure(2, weight = 1)
GUI.grid_rowconfigure(2, weight = 1)


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Set all the motor control pins to outputs
GPIO.setup(1, GPIO.IN)


def Check():
    if GPIO.input(1):
        Text2 = Label(GUI,text='High', fg='#FFFFFF', bg = '#0080FF', padx = 0)
        Text2.grid(row=0,column=1)
    else:
        Text2 = Label(GUI,text='Low', fg='#FFFFFF', bg = '#0080FF', padx = 0)
        Text2.grid(row=0,column=1)
    GUI.after(10, Check)

    
def goodbye():
    GPIO.cleanup()
    #quit()
    

Text1 = Label(GUI,text='Drive Test:', fg='#FFFFFF', bg = '#0080FF', padx = 50, pady = 50)
Text1.grid(row=0,column=0)

Button1 = Button(GUI, text='QUIT', command = goodbye, bg='bisque2', height = 1, width = 12)
Button1.grid(row=2,column=0)

#Button2 = Button(GUI, text='Check', command = Check, bg='bisque2', height = 1, width = 12)
#Button2.grid(row=2,column=1)


GUI.after(10, Check)
GUI.mainloop()

GPIO.cleanup()
