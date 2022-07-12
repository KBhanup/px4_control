import RPi.GPIO as GPIO            # import RPi.GPIO module  
from time import sleep             # lets us have a delay  
GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD  
GPIO.setup(14, GPIO.OUT)           # set GPIO24 as an output   
#GPIO.setup(4, GPIO.OUT)

GPIO.setwarnings(False)

for i in range(1):    
    GPIO.output(14, True)
    sleep(0.000075)                    #when you are sleeping you are not turing it off! but just delaying next command
    GPIO.output(14, False)
    sleep(0.000925)

    GPIO.output(14, True)
    sleep(0.002)                    #when you are sleeping you are not turing it off! but just delaying next command
    GPIO.output(14, False)
    sleep(0.018)

    GPIO.output(14, True)
    sleep(0.002)                    #when you are sleeping you are not turing it off! but just delaying next command
    GPIO.output(14, False)
    sleep(0.018)

  


    #GPIO.output(4, True)
    #sleep(0.002)                   
    #GPIO.output(4, False)
    #sleep(0.018)

    #GPIO.output(4, True)
    #sleep(0.002)                    
    #GPIO.output(4, False)
    #sleep(0.018)

    #GPIO.output(4, True)
    #sleep(0.0015)       
    #GPIO.output(4, False)
    #sleep(0.0185)
    

    #GPIO.output(14, True)
    #sleep(0.0015)       
    #GPIO.output(14, False)
    #sleep(0.0185)
    i+=1
    
       # resets all GPIO ports used by this program  



