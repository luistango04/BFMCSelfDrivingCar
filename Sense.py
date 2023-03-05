import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyrealsense2 as rs
import Setup



global diameterofwheel 
diameterofwheel = 65

class SensingInput:
    def __init__(self,ser,pipeline, GPS=0, IMU=0, INTELLISENSECAMERA=0, V2VLISTENER=0, BNOLISTENER=0):
        self.pipeline = pipeline ##
        self.speed = 0
        self.colorframe = 0
        self.ser = ser
        self.accel = 0
        self.gyro = 0
        self.velo = 0
        self.tilt = 0
        self.counter = 1
        self.imu =  0   ## 0 is no imu 1 is imu
        self.depth_image = 0
        self.errorhandle = [] ##[intellisense, gps, imu, v2v, bno] #error handle

    def Intellsensor(self):  ## captures frame stores in class  # returns 1 if success 0 if fail
        try:
            frames = self.pipeline.wait_for_frames()

            # Get the depth frame

            # Get the depth frame every 5th time
            self.depth_image = frames.get_depth_frame()
            self.depth_image = np.asanyarray(self.depth_image.get_data())
            self.colorframe = frames.get_color_frame()
            self.colorframe = np.asanyarray(self.colorframe.get_data())
            # Reset the counter
            self.counter = 0

            # Get the color frame


            # Convert the depth frame to a NumPy array



            # Convert the color frame to a NumPy array

            self.accel = accel_data(frames[2].as_motion_frame().get_motion_data())

            self.gyro = gyro_data(frames[3].as_motion_frame().get_motion_data())
            #print(self.gyro)
            #print(self.accel)
            # Display the depth image


        except:
            self.errorhandle.append(1) ## Returns 1 if intellisense fails to capture frame
            return 0
        finally:
            return 1
            # Wait for a key press

            # Exit the loop if the 'q' key is pressed



    def velocity(self):
           try:
            # Read a line of data from the serial port
            data = self.ser.readline().decode().strip()

            print(data)
	
            if data:
                # Parse the data to extract the RPM value
                if data.startswith("@5:"):
                    rpm = float(data.split(":")[1].split(";")[0])
      
                    self.velo = rpm * diameterofwheel ## Diameter of wheel
                    print(rpm)
 				
                    return self.velo 
            else:
               print('No data from serial')
               self.velo = 0
               return self.velo 

           except:
		
            print("ERROR IN RPM")
            return 0
    def gettilt(self):

        # Get the current tilt angle
         ##tilt_angle = depth_sensor.get_option(rs.option.camera_angle)

        #print("Tilt angle: ", tilt_angle)
        ##  self.tilt = tilt_angle ## FAILURE
        return 0 ## FAILURE


    def senseall(self):  # runs through all methods to refresh the senses. ## returns bit to sendto debug layer
     	
        error1 = 0 #self.velocity()

        error2 = self.Intellsensor()

        return error1 + error2

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

def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])


