import sys
from SCENE import *
from  Sense import SensingInput
from  Actuation import Actuation
from  Setup import camerainit, pidcarsetting
from VehicleControl import VehicleControl
from unittest.mock import Mock
import serial
import cv2
#from Sign_detection_yolo import detect
import time
sys.path.append('.')

#ser = Mock() ## SET THIS TO SERIAL FOR LIVE!

ser = serial.Serial('/dev/ttyACM1', 19200, timeout=0.1)
ser.flush()

global depthsensor

## BSMF
# Principal AUTHOR: Luis Carpi
# BSMC Project Mobility Challenge
# Self-dRiving RC car in 1 month of less.

# utility imports
# =============================== CONFIG =================================================
enableStream = False
enableCameraSpoof = False
enableRc = True
# =============================== INITIALIZING PROCESSES =================================
allProcesses = list()

global lasttime
global VEHICLE



starttime = time.time()  ## PROOGRAM START

lasttime = starttime
pipeline = camerainit() ### INITIALIzES CAMREA
pidcarsetting(0.1, 0.03, 0.0005, 0.3, 5, ser)  ## SETS UP THE CAR

sensing = SensingInput(ser,pipeline)
global carspeed
carspeed = 0


vehicle = VehicleControl(0,0,0,ser)
vehicle = vehicle(10, 1, 0.01,ser)  ## call function can be used for tesitng


while True:
    print("SENSING")
    sensing.senseall()
    print("SCENING")
    ## TESTING LANE DETECTION
    scene = PScene(sensing)
    ## MAKE A SCENE
    print(scene.lanenode())

#    cv2.waitKey(0)
#    cv2.destroyAllWindows()
     # CONVERT MANEUVERS TO SIGNEL VEHICLE UNDERSTANDS.

	

#    carspeed =sensing.velocity()
#    elapsed_time = lasttime - starttime

#    actuation = Actuation(vehicle,carspeed) ## Get carsoeed from sensor
	    

#    a,b,carspeed = actuation.write_velocity_command(ser,lasttime,starttime)
    #print(carspeed)



    lasttime = time.time()

    time.sleep(.5)


# 1:speed;;
