import RPi.GPIO as GPIO
import time
import math


class Stepper:
    def __init__(self, pin_dir, pin_step, pin_home):
        self._pin_dir = pin_dir
        self._pin_step = pin_step
        self._pin_home = pin_home
        self._pulse_duration = 2 * 1E-6
        self._microsteps = 16
        self._fullsteps_per_mm = 5
        self._position = 0
        self.homing_direction = 0
        self.set_speed(500)
        GPIO.setup(pin_dir, GPIO.OUT)
        GPIO.setup(pin_step, GPIO.OUT)
        GPIO.setup(pin_home, GPIO.IN)
        GPIO.output(pin_step, GPIO.LOW)

    def set_speed(self, mm_per_s):
        step_duration = 1 / (self._fullsteps_per_mm *
                             mm_per_s * self._microsteps)
        self._step_delay = step_duration - self._pulse_duration

    def get_position(self):
        return self._position / self._fullsteps_per_mm

    def set_position(self, new_pos_mm):
        self._position = new_pos_mm * self._fullsteps_per_mm

    def _homed(self):
        # all 10 reads must be ok!
        for i in range(10):
            if GPIO.input(self._pin_home) != GPIO.HIGH:
                return False
        return True

    def home(self):
        GPIO.output(self._pin_dir, self.homing_direction)
        print("homing")
        while not self._homed():
            self._step()
            time.sleep(self._step_delay)
            print("step")
        print("homed")

    def _step(self):
        GPIO.output(self._pin_step, 1)
        time.sleep(self._pulse_duration)
        GPIO.output(self._pin_step, 0)

    def go_to(self, new_position_mm):
        new_position = int(new_position_mm * self._fullsteps_per_mm)
        diff = new_position - self._position
        if diff == 0:
            return
        direction = diff > 0
        GPIO.output(self._pin_dir, direction)
        for i in range(abs(diff)*self._microsteps):
            self._step()
            time.sleep(self._step_delay)
        self._position = new_position
