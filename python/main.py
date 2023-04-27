import RPi.GPIO as GPIO
from Servo import Servo
from Stepper import Stepper
import time


class Pins:
    Servo = 8
    StepperADir = 10
    StepperAStep = 12
    StepperAEndstop = 36
    StepperBDir = 16
    StepperBStep = 18
    StepperBEndstop = 38
    MagnetA = 24
    MagnetB = 26


GPIO.setmode(GPIO.BOARD)
GPIO.setup(Pins.MagnetA, GPIO.OUT)
GPIO.output(Pins.MagnetA, 0)
GPIO.setup(Pins.MagnetB, GPIO.OUT)
GPIO.output(Pins.MagnetB, 0)

while True:
    GPIO.output(Pins.MagnetA, 0)
    time.sleep(1)
    GPIO.output(Pins.MagnetA, 1)
    time.sleep(1)

servo = Servo(Pins.Servo)
stepperX = Stepper(Pins.StepperADir, Pins.StepperAStep, Pins.StepperAEndstop)
stepperY = Stepper(Pins.StepperBDir, Pins.StepperBStep, Pins.StepperBEndstop)
stepperX.homing_direction = 1

stepperX.home()
stepperY.home()
stepperX.set_position(100)

try:
    while True:
        stepperX.go_to(0)
        stepperY.go_to(0)
        servo.go_up()
        time.sleep(1)
        servo.go_down()
        time.sleep(1)
        servo.go_up()
        stepperX.go_to(50)
        stepperY.go_to(50)
        time.sleep(2)
except KeyboardInterrupt:
    servo.stop()
    GPIO.cleanup()
