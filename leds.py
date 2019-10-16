import RPi.GPIO as GPIO
import time

ledTarg = 1

def LedBlue():
    GPIO.output(16, GPIO.HIGH)
    GPIO.output(20, GPIO.LOW)
    
def LedYellow():
    GPIO.output(20, GPIO.HIGH)
    GPIO.output(16, GPIO.LOW)

def ToggleLed(inval):
    if inval == 1:
        LedYellow()
        print('yellow')
        return 0
    else:
        LedBlue()
        return 1


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.IN)


while True:
    if GPIO.input(21):
        ledTarg = ToggleLed(ledTarg)
        time.sleep(1)

