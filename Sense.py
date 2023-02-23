import cv2
import matplotlib.pyplot as plt
import numpy as np
global diameterofwheel

class SensingInput:
    def __init__(self,ser,pipeline, GPS=0, IMU=0, INTELLISENSECAMERA=0, V2VLISTENER=0, BNOLISTENER=0):
        self.speed = 0
        self.colorframe = []
        self.ser = ser
        self.velocity
        self.tilt
        self.pipeline = pipeline

    def import_image(self):  ## captures frame stores in class  # returns 1 if success 0 if fail
        try:
            frames = self.pipeline.wait_for_frames()
            colorframe = frames.get_color_frame()
            color_image = np.asanyarray(colorframe.get_data())
            self.colorframe = color_image
            return 1

        except:
            print("CAMERA NOT FOUND")
            return 0
    def velocity(self):
        while True:
            # Read a line of data from the serial port
            data = self.ser.readline().decode().strip()
            if data:
                # Parse the data to extract the RPM value
                if data.startswith("#5:"):
                    rpm = int(data.split(":")[1].split(";")[0])
                    self.velocity = rpm * diameterofwheel ## Diameter of wheel

                    return rpm

    def gettilt(self):

        depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()

        # Get the current tilt angle
        tilt_angle = depth_sensor.get_option(rs.option.camera_angle)

        print("Tilt angle: ", tilt_angle)
        self.tilt = tilt_angle
        return tilt_angle


    def senseall(self):  # runs through all methods to refresh the senses. ## returns bit to sendto debug layer
        tilt = self.gettilt()
        speed = self.velocity()
        results = self.import_image()
        return tilt,speed

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
