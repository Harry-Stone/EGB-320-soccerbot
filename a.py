import cv2
import numpy as np
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_EXPOSURE, 0.6)                    	# Set the height to 240
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.05) #AUTO?
cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)

while True:



    _,frame = cap.read()
    cv2.imshow("Test",frame)
    cv2.waitKey(1)
