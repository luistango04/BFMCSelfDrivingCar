
import Setup
from Sense import SensingInput
from unittest.mock import Mock
from SCENE import PScene
import time
from Brain import Brain
from VehicleControl import vehiclecontrol
pipeline = Setup.init()
import Actuation
from Actuation import vehiclefree
## Dont forget to turn on the fan sudo sh -c "echo 255 > /sys/devices/pwm-fan/target_pwm"
ser = Mock() ## SET THIS TO SERIAL FOR LIVE!
#ser = serial.Serial('/dev/ttyACM0', 19200, timeout=0.1)
Sense = SensingInput(ser,pipeline)
ser.flush()

def test_fps(object_instance, num_frames=120):
    """
    Test the FPS of an object instance by measuring the time it takes to process a certain number of frames.

    Parameters:
        object_instance (object): An instance of a class that has a "process_frame()" method that takes no arguments and returns None.
        num_frames (int): The number of frames to process.

    Returns:
        float: The estimated FPS of the object instance.
    """
    start_time = time.time()
    for i in range(num_frames):
        object_instance.senseall()
    end_time = time.time()
    elapsed_time = end_time - start_time
    fps = num_frames / elapsed_time
    return fps
#
time.sleep(1) # Give time to fire up camera birghtness
print("CONSTRCT CLASS")
Scene = PScene(Sense)
Brain = Brain(Scene)
vehiclecontrol = vehiclecontrol(Brain,ser,Sense)
Actuation = Actuation.Actuation(vehiclecontrol,ser)

start_time = time.time()
iter = 1
try:
    while True:
        carspeed = .3
        command = f"#1:{carspeed};;\r\n".encode()
    
        #print("PRINTED: " + str(command) + " To console")
        ser.write(command)
        print("SENSING")
        Sense.senseall()
        #cv2.imshow("TEST",Sense.colorframe)\
        Scene = PScene(Sense)
        Scene.makeascene()
        time.sleep(1)
        Brain.update(Scene)
        Brain.perform_action() ## THINK

        print(Brain)
        print()
        #time.sleep(2)
        if (not(vehiclefree) and Brain.emergency == False):
            pass
        else:

            vehiclecontrol.updatefrombrainscene(Brain, Sense)
            Actuation.update(vehiclecontrol)



        #
     #   cv2.waitKey(10000)
        #Scene.lane_detection()
        # test the FPS of the processor object
	
        #cv2.waitKey(5000)
        iter = iter + 1
        pass



except KeyboardInterrupt:
		print("Keyboard interrupt detected. Exiting...")
pipeline.stop()
ser.flush()
carspeed = 0 ## Stop the car
command = f"#1:{carspeed};;\r\n".encode()

end_time = time.time()
time_taken = end_time - start_time
iterations_per_second = iter / time_taken

print("Iterations per second:", iterations_per_second)

