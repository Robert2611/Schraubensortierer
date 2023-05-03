import cv2 as cv
import numpy as np
import camera_calibration

url = "rtsp://homeassistant:pwtypolig@192.168.178.117/stream1"
vcap = cv.VideoCapture(url)
ret, img = vcap.read()
h,  w = img.shape[:2]
# undistort
dst = cv.undistort(img, camera_calibration.mtx,
                   camera_calibration.distortion, None, camera_calibration.newcameramtx)
x, y, w, h = camera_calibration.roi
dst = dst[y:y+h, x:x+w]
cv.imshow("test", dst)
cv.waitKey(0)
