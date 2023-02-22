import sys

from  Actuation import Actuation
from  VehicleControl import VehicleControl
from unittest.mock import Mock
#from Sign_detection_yolo import detect
import time
sys.path.append('.')
import cv2
import matplotlib.pyplot as plt
import numpy as np





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

    def __str__(self):
        return f"yaw_rate: {self.yaw_rate}, lateral_acceleration: {self.lateral_acceleration}, longitudinal_acceleration: {self.longitudinal_acceleration}, speed: {self.speed}, steering_wheel_angle: {self.steering_wheel_angle}, steering_wheel_velocity: {self.steering_wheel_velocity}"


class PScene:
    def __init__(self, SensingInput, camera_resolutionx, camera_resolutiony):
        self.camera_resolution = camera_resolutionx
        self.camera_resolution = camera_resolutiony
        SensingInput.colorframe
        SensingInput.colorframe
        ##    self.intersection_detection = np.zeros((camera_resolution[0], camera_resolution[1]))
        ##    self.midlane = np.zeros((camera_resolution[0], camera_resolution[1]))
        self.sign_trigger = False
        self.intersection_trigger = False
        self.traffic_light_trigger = False
        self.position = 0
        self.SensingInput = SensingInput

    def runobjectdetection(self):
        ## NEED THIS TO RUN
        ## do something to make   SensingInput.colorframe look liek the region of interest .
        # c1 = ((int)(.2 * xresolution), (int)(.3 * yresolution))  ## TOP LEFT
        # c2 = [0, (int)(.7 * yresolution)]  ## BOTTOM LEFT
        # c3 = [xresolution, (int)(.7 * yresolution)]  ## BOTTOM RIGHT
        # c4 = [(int)(.8 * xresolution), (int)(.3 * yresolution)]  # TOP RIGHT
        # ##
        #
        # src = np.float32([c1, c2, c3, c4])
        #
        # # Window to be shown ## NEED ADJUSTMENT  WHEN GO LIVE TO HANDLE THE RESOLUTIONS
        # p1 = [0, 0]  ## TOP LEFT
        # p2 = [0, .7* yresolution]  ## BOTTOM LEFT
        # p3 = [.8*resoltuion, .7*yresolution]  ## BOTTOM RIGHT
        # p4 = [.8* xresolution, 0]  # TOP RIGHT
        #
        # dst = np.float32([p1, p2, p3, p4])
        #
        # # Matrix to warp the image for birdseye window
        # matrix = cv2.getPerspectiveTransform(src, dst)
        #RUN HISTORGRAM LANEDTECTION ON MATRX
        run(matrix,resolutionx,resolutiony)
        ## add additional param for resolution x, resolution y
        ## in case you need it for the tensor function param


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


## DEFINE GLOBALS
# global VEHICLE.speed ## Using globals for now. Might set to global vehicle data object.
global lasttime
global VEHICLE
ser = Mock()

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
    # READ SENSORS
    # Beginning time state
    # sensing.senseall()
    #
    # print(VEHICLE)
    # # MAKE A SCENE
    # perception_scene = PScene(sensing, 1240, 1080)
    # perception_scene.lanenode()
    #
    # # DECIDE WHAT TO DO ON BRAIN
    # brain = Brain(perception_scene)
    # print(brain.get_brain_values())

    # ORCHESTRATE PLANNED MANEUVERS
#    vehicle_control = VehicleControl(brain, perception_scene, VEHICLE)
    #print(vehicle)
    # CONVERT MANEUVERS TO SIGNEL VEHICLE UNDERSTANDS.


    #print(vehicle)
    elapsed_time = lasttime - starttime

    boolbreak = False
    if (elapsed_time > 2 and elapsed_time < 10 and boolbreak == False):
        vehicle.break_execution()
        boolaccel = False

    if (elapsed_time > 10 and elapsed_time < 15 and boolaccel == False):
        print("ACCELEARTION")
        vehicle.accel()
        boolaccel = True

    actuation = Actuation(vehicle,carspeed) ## Get carsoeed from sensor
    #print(actuation.carspeed)
    #print(actuation.velocity)
    a,b,carspeed = actuation.write_velocity_command(ser,lasttime,starttime)
    #print(carspeed)



    lasttime = time.time()

    time.sleep(1)


# 1:speed;;
