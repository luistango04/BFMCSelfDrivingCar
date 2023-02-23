import sys
from  Sense import SensingInput
from  Actuation import Actuation

from Backupdata.VehicleControl import VehicleControl
from unittest.mock import Mock
#from Sign_detection_yolo import detect
import time
sys.path.append('.')

ser = Mock() ## SET THIS TO SERIAL FOR LIVE!

#ser = serial.Serial('/dev/ttyACM0', 19200, timeout=0.1)
ser.flush()



# create a finite state machine

## BSMF
##
# Principal AUTHOR: Luis Carpi
#
# BSMC Project Mobility Challenge
#
# Self-dRiving RC car in 1 month of less.

# utility imports

# =============================== CONFIG =================================================
enableStream = False
enableCameraSpoof = False
enableRc = True

# =============================== INITIALIZING PROCESSES =================================
allProcesses = list()

yresolution = 240  # 720
xresolution = 320  # 420


def camerainit():
    import pyrealsense2 as rs

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, xresolution, yresolution, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline
def pidcarsetting(self, kp, ki, kd, k_t, ser):
    # kp proportional time
    # ki integral coefficient
    # kd derivativecoefficient
    # k_t integraltime
    # ser serial handler

    # write srial fucntions to Car
    ser.write(b'#4:1;;\r\n')
    command = f"#6:{kp};{ki};{kd};{k_t};;\r\n".encode()
    ser.write(command)
    ser.readline()

    # parse read line validate system settings.....

    # hand the system until validation

    # after 10 seconds throw exception and rebooot

    # read serial  back to car
    return 1

    def __str__(self):
        return f"yaw_rate: {self.yaw_rate}, lateral_acceleration: {self.lateral_acceleration}, longitudinal_acceleration: {self.longitudinal_acceleration}, speed: {self.speed}, steering_wheel_angle: {self.steering_wheel_angle}, steering_wheel_velocity: {self.steering_wheel_velocity}"


global lasttime
global VEHICLE

starttime = time.time()  ## PROOGRAM START

lasttime = starttime

sensing = SensingInput()
#pidcarsetting(0.1, 0.03, 0.0005, 0.3, 5, ser))  ## SETS UP THE CAR
global carspeed
carspeed = 0
# pipeline = camerainit() ### INITIALIzES CAMREA
vehicle = VehicleControl(0, 0, 0,ser)

vehicle = vehicle(10, 1, 0.01,ser)  ## call function can be used for tesitng

while True:
    # CONVERT MANEUVERS TO SIGNEL VEHICLE UNDERSTANDS.


    #print(vehicle)
    elapsed_time = lasttime - starttime


    actuation = Actuation(vehicle,carspeed) ## Get carsoeed from sensor
    #print(actuation.carspeed)
    #print(actuation.velocity)
    a,b,carspeed = actuation.write_velocity_command(ser,lasttime,starttime)
    #print(carspeed)



    lasttime = time.time()

    time.sleep(1)


# 1:speed;;
