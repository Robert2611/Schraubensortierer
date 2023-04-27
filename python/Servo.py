import RPi.GPIO as GPIO

class Servo:    
    def __init__(self, pin):
        GPIO.setup(pin, GPIO.OUT)
        self.angle_up = 120
        self.angle_down = 0
        self._servo = GPIO.PWM(pin, 50)
        self._servo.start(self.angle_to_duty(self.angle_up))
        
    def angle_to_duty(self, angle):
        return 2.5 + angle / 180.0 * 10
    
    def go_up(self):
        self._servo.ChangeDutyCycle(self.angle_to_duty(self.angle_up))
        
    def go_down(self):
        self._servo.ChangeDutyCycle(self.angle_to_duty(self.angle_down))
    
    def stop(self):
        self._servo.stop()