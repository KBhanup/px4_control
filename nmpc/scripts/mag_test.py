import time

from magnet_control import MagnetControl

drone_magnet      =  MagnetControl(4)
sensor_magnet_on  =  MagnetControl(14)
sensor_magnet_off =  MagnetControl(15)



#sensor_magnet_on.switchMagnet()
sensor_magnet_off.switchMagnet()

#drone_magnet.droneMagnetEngage()
#drone_magnet.droneMagnetDisengage()

