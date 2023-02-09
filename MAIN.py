
import cv2
import matplotlib.pyplot as plt
import numpy as np

import xml.etree.ElementTree as ET

## BSMF
##
# Principal AUTHOR: Luis Carpi
#
# BSMC Project Mobility Challenge
#
# Self-dRiving RC car in 1 month of less.

class SensingInput:
    def __init__(self, GPS=0, IMU=0, INTELLISENSECAMERA=0, V2VLISTENER=0, BNOLISTENER=0):
        self.GPS = GPS
        self.IMU = IMU
        self.INTELLISENSECAMERA = INTELLISENSECAMERA
        self.V2VLISTENER = V2VLISTENER
        self.BNOLISTENER = BNOLISTENER
        self.ROADIMAGE = 0
        self.SIGNREGION = 0
        self.INTERSECTIONREGION = 0
        self.TRAFFICSTATE = 0
        self.CAR_POS = 0
        self.BNO_POS = 0

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

class VehicleData:
    def __init__(self,yaw_rate,lateral_acceleration,longitudinal_acceleration,speed,steering_wheel_angle,steering_wheel_velocity):
        self.yaw_rate =yaw_rate
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
    def __init__(self,SensingInput, camera_resolutionx,camera_resolutiony):
        self.camera_resolution = camera_resolutionx
        self.camera_resolution = camera_resolutiony
    ##    self.lane_detection = np.zeros(camera_resolutionx, camera_resolutiony)
    ##    self.sign_detection = np.zeros((camera_resolution[0], camera_resolution[1]))
    ##    self.intersection_detection = np.zeros((camera_resolution[0], camera_resolution[1]))
    ##    self.midlane = np.zeros((camera_resolution[0], camera_resolution[1]))
        self.sign_trigger = False
        self.intersection_trigger = False
        self.traffic_light_trigger = False
        self.position = 0
        self.SensingInput = SensingInput

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
                self.sign_trigger, self.intersection_trigger, self.traffic_light_trigger, self.position     )


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

    def __str__(self):
            return f"UNDERCONSTRUCTION"
                # f"break_signal: {self.break_signal}" \
                # f"\nroad_search_signal: {self.road_search_signal}\nswitch_lane_signal: {self.switch_lane_signal}" \
                # f"\nparking_signal: {self.parking_signal}\nlane_following_signal: {self.lane_following_signal}" \
                # f"\nacceleration_signal: {self.acceleration_signal}\nintersection_navigation_signal: {self.intersection_navigation_signal}"


class Actuation:
    def __init__(self, steering, velocity, success=False):
        self.steering = steering
        self.velocity = velocity
        self.success = success

    def get_steering(self):
        return self.steering

    def get_velocity(self):
        return self.velocity

    def __str__(self):
        return "Steering: {} | Velocity: {} | Success: {}".format(self.steering, self.velocity, self.success)

while True:
    # READ SENSORS
    sensing = SensingInput()
    VEHICLE = VehicleData(0.5, 1.0, 2.0, 30, 20, 15)
    print(VEHICLE)
    # MAKE A SCENE
    perception_scene = PScene(sensing,1240,1080)
    print(perception_scene)
    # DECIDE WHAT TO DO ON BRAIN
    brain = Brain(perception_scene)
    print(brain.get_brain_values())
    #ORCHESTRATE PLANNED MANEUVERS
    vehicle_control = VehicleControl(brain,perception_scene,VEHICLE)
    print(vehicle_control)
    # CONVERT MANEUVERS TO SIGNEL VEHICLE UNDERSTANDS.
    actuation = Actuation(vehicle_control.get_steering(),vehicle_control.get_velocity())
    print(actuation)

