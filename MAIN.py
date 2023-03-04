import sys
import Setup


from SCENE import *
from  Sense import SensingInput
global depthsensor

from  Actuation import Actuation
from  Setup import camerainit, pidcarsetting

from VehicleControl import VehicleControl
from unittest.mock import Mock
import serial
import cv2
#from Sign_detection_yolo import detect
import time
sys.path.append('.')
# utility imports
# =============================== CONFIG =================================================
enableStream = False
enableCameraSpoof = False
enableRc = True
# =============================== INITIALIZING PROCESSES =================================
allProcesses = list()
global lasttime
global VEHICLE

ser = Mock() ## SET THIS TO SERIAL FOR LIVE!
#ser = serial.Serial('/dev/ttyACM0', 19200, timeout=0.1)
ser.flush()
pipeline =  Setup.init()
enableStream = False
enableCameraSpoof = False
enableRc = True
# ======================
## BSMF
# Principal AUTHOR: Luis Carpi
# BSMC Project Mobility Challenge
# Self-dRiving RC car in 1 month of less.






start_time = time.time()
lasttime = time.time()

pidcarsetting(0.1, 0.03, 0.0005, 0.3, 5, ser)  ## SETS UP THE CAR

sensing = SensingInput(ser,pipeline)
global carspeed
carspeed = 0


vehicle = VehicleControl(0,0,0,ser)
vehicle = vehicle(10, 1, 0.01,ser)  ## call function can be used for tesitng

try:
	while True:
	    print("SENSING")
	    sensing.senseall()
	    print("SCENING")
	    ## TESTING LANE DETECTION
#	    scene = PScene(sensing)


	    ## MAKE A SCENE
#	    print(scene.lanenode())
	    cv2.imshow("startview", sensing.get_COLORFRAME())
	  
	    cv2.waitKey(0)
	    cv2.destroyAllWindows()
	     # CONVERT MANEUVERS TO SIGNEL VEHICLE UNDERSTANDS.

		

	#    carspeed =sensing.velocity()
	#    elapsed_time = lasttime - starttime

	#    actuation = Actuation(vehicle,carspeed) ## Get carsoeed from sensor
		    

	#    a,b,carspeed = actuation.write_velocity_command(ser,lasttime,starttime)
	    #print(carspeed)

	    lasttime = time.time()  - start_time

	    time.sleep(.5)

except KeyboardInterrupt:
    print("Keyboard interrupt detected. Exiting...")
pipeline.stop()



# 1:speed;;
