import cv2
import numpy as np
import time

url = "rtsp://homeassistant:pwtypolig@192.168.178.117/stream1"
vcap = cv2.VideoCapture()

# take 10 images and save them to harddrive
for i in range(10):
    vcap.open(url)
    ret, img = vcap.read()
    vcap.release()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite("chessboard_{0}.png".format(i), gray)
    #cv2.imshow('img', gray)
    time.sleep(3)
