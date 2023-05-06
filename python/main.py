import camera
import plotter
import cv2
import numpy as np

url = "rtsp://homeassistant:pwtypolig@192.168.178.117/stream1"
calibration_pattern_size = (7, 10)
calibration_pattern_tile_size = 25
sortercam = camera.SorterCamera(url)

if False:
    sortercam.take_calibration_images()
if False:
    sortercam.calibrate(calibration_pattern_size)
if False:
    sortercam.calibrate_position(
        calibration_pattern_size, calibration_pattern_tile_size)
if False:
    img = sortercam.take_undistort_image()
    cv2.imshow("test", img)
    cv2.waitKey(1000)
if True:
    x0, y0 = 250, 238

    p = plotter.Plotter("COM3", 250000)
    p.servo_up()
    p.home()

    def mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pos = sortercam.get_word_coordinates(x, y)
            marlin_pos = round(x0-pos[1], 2), round(y0-pos[0], 2)
            print("{0:0}|{1:0}".format(marlin_pos[0], marlin_pos[1]))
            p.take_at(marlin_pos[0], marlin_pos[1])
            cv2.imshow("image", sortercam.take_undistort_image())

    cv2.imshow("image", sortercam.take_undistort_image())
    cv2.setMouseCallback("image", mouse)
    cv2.waitKey()
