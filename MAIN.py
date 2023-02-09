
import cv2
import matplotlib.pyplot as plt
import numpy as np
import Incaseofemergencyusethis as use

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
    def __init__(self):
        self.yaw_rate = 0
        self.lateral_acceleration = 0
        self.longitudinal_acceleration = 0
        self.speed = 0
        self.steering_wheel_angle = 0
        self.steering_wheel_velocity = 0

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


class PScene:
    def __init__(self, camera_resolutionx,y):
        self.camera_resolution = camera_resolutionx
        self.camera_resolution = camera_resolutiony
        self.lane_detection = np.zeros((camera_resolution[0], camera_resolution[1]))
        self.sign_detection = np.zeros((camera_resolution[0], camera_resolution[1]))
        self.intersection_detection = np.zeros((camera_resolution[0], camera_resolution[1]))
        self.midlane = np.zeros((camera_resolution[0], camera_resolution[1]))
        self.sign_trigger = False
        self.intersection_trigger = False
        self.traffic_light_trigger = False
        self.position = 0

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


class Brain:
    def __init__(self, num_scenes):
        self.num_scenes = num_scenes
        self.SceneArray = [PScene((10, 10)) for i in range(num_scenes)]
        self.Break = 0
        self.RoadSearch = 0
        self.SwitchLane = 0
        self.Parking = 0
        self.LaneFollowing = 0
        self.Acceleration = 0
        self.IntersectionNavigation = 0
        self.PlannedActivities = 0

    def get_brain_values(self):
        return [self.Break, self.RoadSearch, self.SwitchLane, self.Parking, self.LaneFollowing, self.Acceleration, self.IntersectionNavigation, self.PlannedActivities]




class VehicleControl:
    def __init__(self, brain, pscene, vehicle_data):
        self.brain = brain
        self.pscene = pscene
        self.vehicle_data = vehicle_data

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


class Actuation:
    def __init__(self, vehicle_control):
        self.vehicle_control = vehicle_control

    def set_steering(self, steering_value):
        # Implement the steering function using the `steering_value`
        pass

    def set_velocity(self, velocity_value):
        # Implement the velocity function using the `velocity_value`
        pass


