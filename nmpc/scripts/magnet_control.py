#!/usr/bin/python
import RPi.GPIO as GPIO
from time import sleep


class MagnetControl:
    def __init__(self, pin, rate=50):
        self.pin = pin
        self.rate = rate
        self.dt = 1.0 / self.rate

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.setwarnings(False)

    def fakePWMSignal(self, width_high, time_on=0.5):
        loops = int(time_on / self.dt)

        for i in range(loops):
            GPIO.output(self.pin, True)
            sleep(width_high)
            GPIO.output(self.pin, False)
            sleep(self.dt - width_high)

    def droneMagnetEngage(self,):
        # Send engage signal
        self.fakePWMSignal(0.002)

        # Send neutral signal
        self.fakePWMSignal(0.0015)

    def droneMagnetDisengage(self,):
        # Send disengage signal
        self.fakePWMSignal(0.001)

        # Send neutral signal
        self.fakePWMSignal(0.0015)

    # For sensor magnet
    def switchMagnet(self,):
        GPIO.output(self.pin, True)
        sleep(2)
        GPIO.output(self.pin, False)
