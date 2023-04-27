from picamera import PiCamera
from time import sleep

camera = PiCamera()
try:
    camera.start_preview()
    while True:
        sleep(10)
except KeyboardInterrupt:
    camera.stop_preview()
