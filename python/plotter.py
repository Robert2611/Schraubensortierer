import serial
import time


class Plotter():
    feedrate = 10000
    servo_down_pos = 1500
    servo_up_pos = 2200
    __timeout = 0.2
    verbose = False

    def __init__(self, port, baudrate):
        if self.verbose:
            print("Connect")
        self.__ser = serial.Serial(port, baudrate, timeout=self.__timeout)
        time.sleep(2)
        self.__serial_read_available()

    def __serial_read_available(self):
        line = True
        while line != b'':
            line = self.__ser.readline()

    def __await_marlin_command(self, cmd):
        if self.verbose:
            print("Sending '{0}'".format(cmd))
        cmd += "\n"
        self.__ser.write(cmd.encode())
        line = ""
        while "ok" not in line:
            line = self.__ser.readline().decode()
            if self.verbose:
                print("Received '{0}'".format(line))

    def dwell(self, seconds):
        self.__await_marlin_command("G4 P{0}".format(int(seconds * 1000)))

    def move(self, x, y):
        self.__await_marlin_command(
            "G1 X{0} Y{1} F{2}".format(x, y, self.feedrate))

    def magnet(self, on):
        self.__await_marlin_command("M106 P0" if on else "M107 P0")

    def servo_up(self):
        # make sure to execute in order with G-Commands
        self.__await_marlin_command("M400")
        self.__await_marlin_command("M280 P0 S{0}".format(self.servo_up_pos))

    def servo_down(self):
        # make sure to execute in order with G-Commands
        self.__await_marlin_command("M400")
        self.__await_marlin_command("M280 P0 S{0}".format(self.servo_down_pos))

    def home(self):
        self.__await_marlin_command("G28 X Y")

    def take_at(self, x, y):
        self.move(x, y)
        self.servo_down()
        self.magnet(True)
        self.dwell(0.5)
        self.servo_up()

    def drop_at(self, x, y):
        self.move(x, y)
        self.magnet(False)
        self.dwell(0.5)
        # shake off the screw
        self.move(x+1, y)
        self.move(x, y)
        self.move(x, y+1)
        self.move(x, y)
        self.dwell(0.5)
