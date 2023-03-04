import sys
import Setup

from SCENE import *
from  Sense import *
from  Actuation import Actuation
from  Setup import camerainit, pidcarsetting

from VehicleControl import VehicleControl
from unittest.mock import Mock
import serial
import cv2
#from Sign_detection_yolo import detect
import time

global pipeline


sys.path.append('.')

ser = Mock() ## SET THIS TO SERIAL FOR LIVE!

#ser = serial.Serial('/dev/ttyACM0', 19200, timeout=0.1)
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

#sensing = SensingInput(ser,pipeline)
global carspeed
carspeed = 0
#
pipeline = Setup.init()

# Create a RealSense pipeline
sensing = SensingInput(ser,pipeline)

try:
    while True:
        print("SENSING")
        sensing.senseall()
        print("SCENING")
        ## TESTING LANE DETECTION
        scene = PScene(sensing)
        scene.lanenode()



        ## MAKE A SCENE
        #	    print(scene.lanenode())
#        cv2.imshow("startview", sensing.get_COLORFRAME())

        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # CONVERT MANEUVERS TO SIGNEL VEHICLE UNDERSTANDS.

        #    carspeed =sensing.velocity()
        #    elapsed_time = lasttime - starttime

        #    actuation = Actuation(vehicle,carspeed) ## Get carsoeed from sensor

        #    a,b,carspeed = actuation.write_velocity_command(ser,lasttime,starttime)
        # print(carspeed)

#        lasttime = time.time() - start_time

        time.sleep(.5)

except KeyboardInterrupt:
    print("Keyboard interrupt detected. Exiting...")
pipeline.stop()
