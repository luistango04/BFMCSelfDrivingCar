import unittest
from unittest.mock import Mock
import my_module
import time
global lastspeed
global lasttime
global starttime
def write_velocity_command(velocity,acceleration,ser,lastspeed): ## Acceleration in M/S
        step = time.time() - lasttime
        carspeed = lastspeed
        if (carspeed < velocity):
            carspeed = min(velocity, carspeed + (acceleration * step))
        elif (carspeed > velocity):
            carspeed = max(velocity, carspeed - (acceleration * step))

        command = f"#1:{carspeed};;\r\n".encode()
        print("Current Speed:" + str(round(carspeed)) + "  target =" + str(round(velocity)) + "  seconds  elapsed :" + str(time.time()-starttime))
        print("PRINTED: " + str(command) + " To console")
        ser.write(command)

        if (carspeed == velocity):
            return 1, carspeed, time.time()
        return 0, carspeed, time.time()



ser = Mock()
starttime  = time.time()
lasttime = starttime
lastspeed =  20 ## SENSOR VELOCITY
while True:
    result,lastspeed,lasttime = write_velocity_command(10, 1, ser,lastspeed)
    time.sleep(1)


