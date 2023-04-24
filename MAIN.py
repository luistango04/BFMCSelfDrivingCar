import Setup
from Setup import DEBUG_MODE, JETSON_MODE,NAZRUL_MODE,SERIALDEBUG,pipeline
from Sense import SensingInput
from unittest.mock import Mock
from SCENE import PScene
import time
from Brain import Brain
from MQTTGenericClient import MQTTGenericClient
from GenericJsonReader import GenericJsonReader
from VehicleControl import vehiclecontrol
from  Actuation import bothfree,velofree,steeringfree
import Actuation
import cv2
import serial
import random
## Dont forget to turn on the fan sudo sh -c "echo 255 > /sys/devices/pwm-fan/target_pwm"



jsonReader = None# GenericJsonReader("MQTTVehicleControlMessages.json")
#jsonReader = GenericJsonReader("MQTTVehicleControlMessages.json")
#mqttControlMessage = MQTTGenericClient(f"jetsonCar{random.randint(0, 1000)}", 1, jsonReader)
#mqttControlMessage.start_client()
#mqttControlMessage.subscribe(Setup.BFMC_MQTT_CONTROL_TOPIC)
system_start = True

if(SERIALDEBUG):
    ser = Mock()
else:
    try:
        ser = serial.Serial('/dev/ttyACM0', 19200, timeout=0.1)
    except:
        ser = serial.Serial('/dev/ttyACM1', 19200, timeout=0.1)

model = Setup.init(ser)
Sense = SensingInput(ser)
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
time.sleep(1)  # Give time to fire up camera birghtness
Scene = PScene(Sense)

Brain = Brain(Scene)
vehiclecontrol = vehiclecontrol(Brain, ser, Sense)
Act = Actuation.Act(vehiclecontrol, ser)

start_time = time.time()
iter = 1
carspeed = .2
command = f"#1:{carspeed};;\r\n".encode()


#ser.write(command)
try:
    while (iter < 100000):

        iter = iter + 1
        #print("PRINTED: " + str(Act.steeringstatus) + " To console")
        print(iter)
        # print("SENSING")
        # cv2.imshow("TEST",Sense.colorframe)\
        Scene = PScene(Sense)
        if(JETSON_MODE):
            Sense.senseall()
            Scene.makeascene(model)
 	 
        Brain.update(Scene)
        Brain.perform_action()  ## THINK
        if(DEBUG_MODE):
            #print("DEBUG MODE")
            print(Scene)
            print("BRAIN GOT")
            print(Brain)
		
            print("Speed", vehiclecontrol.velocommands)
            time.sleep(.1)



            cv2.waitKey(500)

        # time.sleep(2)
        if (not (Act.steeringstatus) and not(Act.velocitystatus) and Brain.override == False):
            print("carnotready")
            pass
        else:
            print("carready")
            vehiclecontrol.updatefrombrainscene(Brain, Sense)
            Act.update(vehiclecontrol)

        #
        #   cv2.waitKey(10000)
        # Scene.lane_detection()
        # test the FPS of the processor object

        # cv2.waitKey(5000)

        pass



except KeyboardInterrupt:
    print("Keyboard interrupt detected. Exiting...")
    
#    ser.flush()
#    carspeed = 0  ## Stop the car
#    command = f"#1:{carspeed};;\r\n".encode()
#    ser.write(command)

end_time = time.time()
time_taken = end_time - start_time
iterations_per_second = iter / time_taken

print("Iterations per second:", iterations_per_second)
pipeline.stop()


