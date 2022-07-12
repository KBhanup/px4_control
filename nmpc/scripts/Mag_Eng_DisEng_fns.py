#!/usr/bin/python
import RPi.GPIO as GPIO            # import RPi.GPIO module  
from time import sleep             # lets us have a delay
import rospy as rp  
#GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD  
#GPIO.setup(27, GPIO.OUT)           # set GPIO24 as an output   





class Mag:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.setwarnings(False)
        self.Sensor_Mag = 17
        self.Drone_Mag = 4
      

    
    def engage(self):
        for i in range(1,5000):    # Change this to a condition or call
            GPIO.output(self.pin , True)
            sleep(0.002)                    #when you are sleeping you are not turing it off! but just delaying next command
            GPIO.output(27, False)
            sleep(0.018)

            GPIO.output(self.pin , True)
            sleep(0.002)                    
            GPIO.output(self.pin , False)
            sleep(0.018)

            GPIO.output(self.pin , True)
            sleep(0.002)                    
            GPIO.output(self.pin , False)
            sleep(0.018)
            rp.loginfo('Magnet Engaged')
            GPIO.output(self.pin , True)
            sleep(0.0015)                  
            GPIO.output(self.pin , False)
            sleep(0.0185)

            GPIO.output(self.pin , True)
            sleep(0.0015)                    
            GPIO.output(self.pin , False)
            sleep(0.0185)

            GPIO.output(self.pin , True)
            sleep(0.0015)                    
            GPIO.output(self.pin , False)
            sleep(0.0185)
        rp.loginfo('Magnet Neutral')
        i+=1

        

    def disengage(self):
        for i in range(1,5000):    # Change this to a condition or call
            GPIO.output(self.pin , True)
            sleep(0.001)                    
            GPIO.output(self.pin , False)
            sleep(0.019)

            GPIO.output(self.pin , True)
            sleep(0.001)                    
            GPIO.output(self.pin , False)
            sleep(0.019)

            GPIO.output(self.pin , True)
            sleep(0.001)                    
            GPIO.output(self.pin , False)
            sleep(0.019)
            rp.loginfo('Magnet Disengaged')
            GPIO.output(self.pin , True)
            sleep(0.0015)                    
            GPIO.output(self.pin , False)
            sleep(0.0185)

            GPIO.output(self.pin , True)
            sleep(0.0015)                    
            GPIO.output(self.pin , False)
            sleep(0.0185)

            GPIO.output(self.pin , True)
            sleep(0.0015)                    
            GPIO.output(self.pin , False)
            sleep(0.0185)
        rp.loginfo('Magnet Neutral')
        


    
    
        # resets all GPIO ports used by this program  
        # except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt  
        # GPIO.cleanup()       
