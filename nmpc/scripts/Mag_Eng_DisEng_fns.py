#!/usr/bin/python
import RPi.GPIO as GPIO            # import RPi.GPIO module
from time import sleep             # lets us have a delay
import rospy as rp



class Mag:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.setwarnings(False)
#        self.Sensor_MagON = 14
#        self.Sensor_MagOFF = 15
#        self.Drone_Mag = 4

    def dr_Magengage(self):
        GPIO.output(self.pin, True)
        # when you are sleeping you are not turing it off! but just delaying next command
        sleep(0.002)
        GPIO.output(self.pin, False)
        sleep(0.018)

        GPIO.output(self.pin, True)
        sleep(0.002)
        GPIO.output(self.pin, False)
        sleep(0.018)
        GPIO.output(self.pin, True)
        sleep(0.002)
        GPIO.output(self.pin, False)
        sleep(0.018)
        rp.loginfo('Magnet Engaged')

        GPIO.output(self.pin, True)
        sleep(0.0015)
        GPIO.output(self.pin, False)
        sleep(0.0185)
        GPIO.output(self.pin, True)
        sleep(0.0015)
        GPIO.output(self.pin, False)
        sleep(0.0185)
        GPIO.output(self.pin, True)
        sleep(0.0015)
        GPIO.output(self.pin, False)
        sleep(0.0185)
        rp.loginfo('Magnet Neutral')


    def dr_Magdisengage(self):
        GPIO.output(self.pin, True)
        sleep(0.001)
        GPIO.output(self.pin, False)
        sleep(0.019)

        GPIO.output(self.pin, True)
        sleep(0.001)
        GPIO.output(self.pin, False)
        sleep(0.019)
        GPIO.output(self.pin, True)
        sleep(0.001)
        GPIO.output(self.pin, False)
        sleep(0.019)
        rp.loginfo('Magnet Disengaged')

        GPIO.output(self.pin, True)
        sleep(0.0015)
        GPIO.output(self.pin, False)
        sleep(0.0185)
        GPIO.output(self.pin, True)
        sleep(0.0015)
        GPIO.output(self.pin, False)
        sleep(0.0185)
        GPIO.output(self.pin, True)
        sleep(0.0015)
        GPIO.output(self.pin, False)
        sleep(0.0185)
        rp.loginfo('Magnet Neutral')

    def Sn_Magengage(self):
        GPIO.output(self.pin, True)
        # when you are sleeping you are not turing it off! but just delaying next command
        sleep(2)
        GPIO.output(self.pin, False)
        rp.loginfo('Sensor Magnet Engaged')

    def Sn_Magdisengage(self):
        GPIO.output(self.pin, True)
        # when you are sleeping you are not turing it off! but just delaying next command
        sleep(2)
        GPIO.output(self.pin, False)
        rp.loginfo('Sensor Magnet Dis-Engaged')

        # resets all GPIO ports used by this program
        # except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt
        # GPIO.cleanup()
