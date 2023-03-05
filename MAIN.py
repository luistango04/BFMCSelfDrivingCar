import pyrealsense2 as rs
import numpy as np
import cv2
import Setup
from Sense import SensingInput
from unittest.mock import Mock
from SCENE import PScene
import time
from Brain import Brain
from VehicleControl import vehiclecontrol
pipeline = Setup.init()
from Actuation import Actuation

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


test_fps(Sense, 120)
# test the FPS of the processor object
fps = test_fps(Sense)

print(f"Estimated FPS: {fps:.2f}")

Scene = PScene(Sense)
Brain = Brain(Scene)
vehiclecontrol = vehiclecontrol(Brain,ser,Sense)
Actuation = Actuation(vehiclecontrol,ser)

start_time = time.time()

try:
    while True:
        print("SENSING")
        Sense.senseall()
        Scene = PScene(Sense)


        Scene.lane_detection()
        Brain.update_from_scene(Scene)
        print(Brain)
        vehiclecontrol.updatefrombrainscene(Brain,Sense)
        vehiclecontrol.lanefollow()
        Actuation.update(vehiclecontrol)
        Actuation.write_angle_command()

        #
       # cv2.waitKey(5000)
        #Scene.lane_detection()
        # test the FPS of the processor object


        pass



except KeyboardInterrupt:
		print("Keyboard interrupt detected. Exiting...")
pipeline.stop()

end_time = time.time()
time_taken = end_time - start_time
iterations_per_second = 1 / time_taken

print("Iterations per second:", iterations_per_second)

