#!/usr/bin/python3

import numpy as np
import cv2

cam = cv2.VideoCapture(0)
while True:
    ret_val, img = cam.read()
    if ret_val:
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)
        # h_thresholded = cv2.threshold(h,50,255,cv2.THRESH_BINARY)
        indices = np.all(h<30)
        h[indices]=0
        cv2.imshow('graustufenbild', h)
    else:
        print("Oh no meine Kamera is schrott.")
    if cv2.waitKey(1) == 27: 
        break  # esc to quit
cv2.destroyAllWindows()