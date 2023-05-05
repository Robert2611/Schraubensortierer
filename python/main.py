import cam
import cv2

url = "rtsp://homeassistant:pwtypolig@192.168.178.117/stream1"
calibration_pattern_size = (7, 10)
calibration_pattern_tile_size = 25
sortercam = cam.SorterCam(url)
# sortercam.take_calibration_images()
# sortercam.calibrate(calibration_pattern_size)
# sortercam.calibrate_position(
#    calibration_pattern_size, calibration_pattern_tile_size)
img = sortercam.take_undistort_image()
cv2.imshow("test", img)
cv2.waitKey(1000)
