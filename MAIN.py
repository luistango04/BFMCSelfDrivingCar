import sys
from unittest.mock import Mock

sys.path.append('.')
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time

#### CAR GLOBAL MAX
global maxsteering
maxsteering = 23
global minsteering
minsteering = -23
global maxspeed
maxspeed = .3
global minspeed
minspeed = -.3


# create a finite state machine
class FSM:
    def __init__(self, states, start_state):
        self.states = states
        self.state = start_state

    def run(self, input):
        (new_state, output) = self.states[self.state](input)
        self.state = new_state
        return output


import xml.etree.ElementTree as ET

## BSMF
##
# Principal AUTHOR: Luis Carpi
#
# BSMC Project Mobility Challenge
#
# Self-dRiving RC car in 1 month of less.

from multiprocessing import Pipe, Process, Event
from src.hardware.serialhandler.SerialHandlerProcess import SerialHandlerProcess

# utility imports
from src.utils.camerastreamer.CameraStreamerProcess import CameraStreamerProcess
from src.utils.remotecontrol.RemoteControlReceiverProcess import RemoteControlReceiverProcess

# =============================== CONFIG =================================================
enableStream = False
enableCameraSpoof = False
enableRc = True

# =============================== INITIALIZING PROCESSES =================================
allProcesses = list()

yresolution = 240  # 720
xresolution = 320  # 420
import numpy as np
import cv2
import pyrealsense2 as rs
from lanedetection import perspectiveWarp, processImage, plotHistogram, slide_window_search, general_search, \
    measure_lane_curvature, draw_lane_lines, offCenter, addText
import lanedetection


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


class SensingInput:
    def __init__(self, GPS=0, IMU=0, INTELLISENSECAMERA=0, V2VLISTENER=0, BNOLISTENER=0):
        self.speed = 0
        self.colorframe = []

    def import_image(self):  ## captures frame stores in class  # returns 1 if success 0 if fail
        try:
            frames = pipeline.wait_for_frames()
            colorframe = frames.get_color_frame()
            color_image = np.asanyarray(colorframe.get_data())
            self.colorframe = color_image
            return 1

        except:
            print("CAMERA NOT FOUND")
            return 0

    def senseall(self):  # runs through all methods to refresh the senses. ## returns bit to sendto debug layer
        results = self.import_image()
        return results

    def get_COLORFRAME(self):
        return self.colorframe

    def get_GPS(self):
        return self.GPS

    def get_IMU(self):
        return self.IMU

    def get_INTELLISENSECAMERA(self):
        return self.INTELLISENSECAMERA

    def get_V2VLISTENER(self):
        return self.V2VLISTENER

    def get_BNOLISTENER(self):
        return self.BNOLISTENER

    def get_ROADIMAGE(self):
        return self.ROADIMAGE

    def get_SIGNREGION(self):
        return self.SIGNREGION

    def get_INTERSECTIONREGION(self):
        return self.INTERSECTIONREGION

    def get_TRAFFICSTATE(self):
        return self.TRAFFICSTATE

    def get_CAR_POS(self):
        return self.CAR_POS

    def get_BNO_POS(self):
        return self.BNO_POS


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


class VehicleData:
    def __init__(self, yaw_rate, lateral_acceleration, longitudinal_acceleration, speed, steering_wheel_angle,
                 steering_wheel_velocity):
        self.yaw_rate = yaw_rate
        self.lateral_acceleration = lateral_acceleration
        self.longitudinal_acceleration = longitudinal_acceleration
        self.speed = speed
        self.steering_wheel_angle = steering_wheel_angle
        self.steering_wheel_velocity = steering_wheel_velocity

    def get_yaw_rate(self):
        return self.yaw_rate

    def get_lateral_acceleration(self):
        return self.lateral_acceleration

    def get_longitudinal_acceleration(self):
        return self.longitudinal_acceleration

    def get_speed(self):
        return self.speed

    def get_steering_wheel_angle(self):
        return self.steering_wheel_angle

    def get_steering_wheel_velocity(self):
        return self.steering_wheel_velocity

    def __str__(self):
        return f"yaw_rate: {self.yaw_rate}, lateral_acceleration: {self.lateral_acceleration}, longitudinal_acceleration: {self.longitudinal_acceleration}, speed: {self.speed}, steering_wheel_angle: {self.steering_wheel_angle}, steering_wheel_velocity: {self.steering_wheel_velocity}"


class PScene:
    def __init__(self, SensingInput, camera_resolutionx, camera_resolutiony):
        self.camera_resolution = camera_resolutionx
        self.camera_resolution = camera_resolutiony
        self.lane_detection = SensingInput.colorframe
        self.sign_detection = SensingInput.colorframe
        ##    self.intersection_detection = np.zeros((camera_resolution[0], camera_resolution[1]))
        ##    self.midlane = np.zeros((camera_resolution[0], camera_resolution[1]))
        self.sign_trigger = False
        self.intersection_trigger = False
        self.traffic_light_trigger = False
        self.position = 0
        self.SensingInput = SensingInput

    def lanenode(self):
        try:
            birdView, birdViewL, birdViewR, minverse = perspectiveWarp(self.lane_detection)

            # Apply image processing by calling the "processImage()" function
            # Then assign their respective variables (img, hls, grayscale, thresh, blur, canny)
            # Provide this function with:
            # 1- an already perspective warped image to process (birdView)
            img, hls, grayscale, thresh, blur, canny = processImage(birdView)
            imgL, hlsL, grayscaleL, threshL, blurL, cannyL = processImage(birdViewL)
            imgR, hlsR, grayscaleR, threshR, blurR, cannyR = processImage(birdViewR)

            # Plot and display the histogram by calling the "get_histogram()" function
            # Provide this function with:
            # 1- an image to calculate histogram on (thresh)
            hist, leftBase, rightBase, midpoint = plotHistogram(thresh)
            # # print(rightBase - leftBase)
            # plt.plot(hist)

            # plt.show()

            #

            # (frame)
            ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, hist)
            plt.plot(left_fit)
            # plt.show()
            #
            #
            draw_info = general_search(thresh, left_fit, right_fit)
            # plt.show()
            #
            #

            curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx)
            #
            #
            # # Filling the area of detected lanes with green
            meanPts, result = draw_lane_lines(self.lane_detection, thresh, minverse, draw_info)
            #
            #
            deviation, directionDev = offCenter(meanPts, self.lane_detection)
            #
            #
            # # Adding text to our final image
            finalImg = addText(result, curveRad, curveDir, deviation, directionDev)
            #
            # # Displaying final image
            # cv2.imshow("Final", finalImg)
            #      out.write(finalImg)
            #

            # Wait for the ENTER key to be pressed to stop playback
            print("FYI MIDPOINT DETECTED WAS:" + str(midpoint))

        except Exception as e:
            elapsed_time = lasttime - starttime
            print("Error occurred at time: {:.2f} seconds".format(elapsed_time))
            print("Error message:", e)

    def get_camera_resolution(self):
        return self.camera_resolution

    def get_lane_detection(self):
        return self.lane_detection

    def get_sign_detection(self):
        return self.sign_detection

    def get_intersection_detection(self):
        return self.intersection_detection

    def get_midlane(self):
        return self.midlane

    def get_sign_trigger(self):
        return self.sign_trigger

    def get_intersection_trigger(self):
        return self.intersection_trigger

    def get_traffic_light_trigger(self):
        return self.traffic_light_trigger

    def get_position(self):
        return self.position

    def get_vehicle_data(self):
        return self.vehicle_data

    def __str__(self):
        return 'midlane: {}\nsign_trigger: {}\nintersection_trigger: {}\ntraffic_light_trigger: {}\nposition: {}\n'.format(
            0,
            self.sign_trigger, self.intersection_trigger, self.traffic_light_trigger, self.position)


class Brain:
    def __init__(self, xml_data=None):
        self.break_trigger = 0
        self.road_search_trigger = 0
        self.switch_lane_trigger = 0
        self.parking_trigger = 0
        self.lane_following_trigger = 0
        self.acceleration_trigger = 0
        self.intersection_navigation_trigger = 0

        # if xml_data is not None:
        #     root = ET.fromstring(xml_data)
        #     for child in root:
        #         if child.tag == "break":
        #             self.break_trigger = int(child.attrib["value"])
        #         elif child.tag == "road_search":
        #             self.road_search_trigger = int(child.attrib["value"])
        #         elif child.tag == "switch_lane":
        #             self.switch_lane_trigger = int(child.attrib["value"])
        #         elif child.tag == "parking":
        #             self.parking_trigger = int(child.attrib["value"])
        #         elif child.tag == "lane_following":
        #             self.lane_following_trigger = int(child.attrib["value"])
        #         elif child.tag == "acceleration":
        #             self.acceleration_trigger = int(child.attrib["value"])
        #         elif child.tag == "intersection_navigation":
        #             self.intersection_navigation_trigger = int(child.attrib["value"])

    def get_brain_values(self):
        return [self.break_trigger,
                self.road_search_trigger,
                self.switch_lane_trigger,
                self.parking_trigger,
                self.lane_following_trigger,
                self.acceleration_trigger,
                self.intersection_navigation_trigger,
                ]


class VehicleControl:
    def __init__(self, brain, pscene, vehicle_data):
        self.brain = brain
        self.pscene = pscene
        self.vehicle_data = vehicle_data
        self.steering = 0
        self.velocity = 0
        self.sterringbound = [-20, 20]  # degrees

    def get_steering(self):
        return self.steering

    def get_velocity(self):
        return self.velocity

    def break_execution(self):
        # Get break trigger value from brain object
        break_trigger = self.brain.break_trigger

        # Execute break function based on trigger value
        if break_trigger:
            # Break function implementation here
            pass

    def road_search_execution(self):
        # Get road search trigger value from brain object
        road_search_trigger = self.brain.road_search_trigger

        # Execute road search function based on trigger value
        if road_search_trigger:
            # Road search function implementation here
            pass

    def switch_lane_execution(self):
        # Get switch lane trigger value from brain object
        switch_lane_trigger = self.brain.switch_lane_trigger

        # Execute switch lane function based on trigger value
        if switch_lane_trigger:
            # Switch lane function implementation here
            pass

    def parking_execution(self):
        # Get parking trigger value from brain object
        parking_trigger = self.brain.parking_trigger

        # Execute parking function based on trigger value

    def lanefollow(self, xcenter_lane=None, xcenter_image=None):
        # stay and correct to center of Lane
        Kp = 0.1  # Proportional gain
        Kd = 0.01  # Derivative gain

        # Define initial error and derivative of error
        error = xcenter_lane - xcenter_image
        prev_error = 0

        # PD controller
        # while True:
        # Update error and derivative of error
        error = xcenter_lane - xcenter_image
        error_diff = error - prev_error
        prev_error = error

        # Calculate steering angle
        angle = Kp * error + Kd * error_diff

        # Limit the steering angle to the maximum and minimum values
        angle = max(min_angle, min(max_angle, angle))

        # Apply steering angle to the vehicle

        # speed is lastspeed
        return angle, VEHICLE.speed

        # return to cruising speed

        # Execute parking function based on trigger value

    def __str__(self):
        return f"UNDERCONSTRUCTION"
        # f"break_signal: {self.break_signal}" \
        # f"\nroad_search_signal: {self.road_search_signal}\nswitch_lane_signal: {self.switch_lane_signal}" \
        # f"\nparking_signal: {self.parking_signal}\nlane_following_signal: {self.lane_following_signal}" \
        # f"\nacceleration_signal: {self.acceleration_signal}\nintersection_navigation_signal: {self.intersection_navigation_signal}"


class Actuation:
    def __init__(self, steering, velorate, accelerations):
        self.steering = steering
        self.velocity = velorate * maxspeed
        self.acceleration = .05

    def write_velocity_command(self, ser, velocity):
        """
        This function writes a velocity command to the given serial port `ser` with the specified `velocity` and `acceleration`.

        Parameters:
            velocity (float): The target velocity to be set.
            acceleration (float): The acceleration of the device in M/S.
            ser (serial.Serial): The serial port to write the command to.
            VEHICLE.speed (float): The last recorded speed of the device.

        Returns:
            Tuple: A tuple containing the following values:
                int:
                    1 if the speed has reached the target velocity,
                    0 if the speed is still accelerating or decelerating,
                    -1 if an error occurred.
                float: The current speed of the device.
                float: The time elapsed since the start of the function.
        """
        # Calculate the time step since the last update

        # Initialize the current speed with the last recorded speed#

        carspeed = velocity

        step = time.time() - lasttime

        # If the current speed is less than the target velocity, increase the speed
        if (carspeed < self.velocity):
            carspeed = min(velocity, carspeed + (self.acceleration * step))  ## DAMPENINING

        # If the current speed is greater than the target velocity, decrease the speed
        elif (carspeed > self.velocity):
            carspeed = max(self.velocity, carspeed - (self.acceleration * step))

        # Encode the command string and write it to the serial port
        command = f"#1:{carspeed};;\r\n".encode()
        print("Current Speed:" + str(round(carspeed)) + "  target =" + str(
            round(self.velocity)) + "  seconds  elapsed :" + str(time.time() - starttime))
        print("PRINTED: " + str(command) + " To console")
        ser.write(command)

        # If the current speed is equal to the target velocity, return 1
        if (carspeed == self.velocity):
            return 1, time.time()

        # Otherwise, return 0 to indicate that the speed is still accelerating or decelerating
        return 0, time.time()

    def get_steering(self):
        return self.steering

    def get_velocity(self):
        return self.velocity

    def __str__(self):
        return " Input Steering: {} | Velocity: {}".format(self.steering, self.velocity, )


## DEFINE GLOBALS
# global VEHICLE.speed ## Using globals for now. Might set to global vehicle data object.
global lasttime
global VEHICLE
ser = Mock()

starttime = time.time()  ## PROOGRAM START

lasttime = starttime
VEHICLE = VehicleData(0.5, 1.0, 2.0, .5, 20, 15)  ## SAMPLE DATA

sensing = SensingInput()
print(pidcarsetting(0.1, 0.03, 0.0005, 0.3, 5, ser))  ## SETS UP THE CAR

# pipeline = camerainit() ### INITIALIzES CAMREA

while True:
    # READ SENSORS
    # Beginning time state
    sensing.senseall()

    print(VEHICLE)
    # MAKE A SCENE
    perception_scene = PScene(sensing, 1240, 1080)
    perception_scene.lanenode()

    # DECIDE WHAT TO DO ON BRAIN
    brain = Brain(perception_scene)
    print(brain.get_brain_values())

    # ORCHESTRATE PLANNED MANEUVERS
    vehicle_control = VehicleControl(brain, perception_scene, VEHICLE)
    print(vehicle_control)
    # CONVERT MANEUVERS TO SIGNEL VEHICLE UNDERSTANDS.
    actuation = Actuation(vehicle_control.get_steering(), vehicle_control.get_velocity(), 4)
    a, c = actuation.write_velocity_command(ser, VEHICLE.get_speed())
    print(actuation)
    time.sleep(2)

# 1:speed;;
