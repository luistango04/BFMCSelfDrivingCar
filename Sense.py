import cv2
import matplotlib.pyplot as plt
import numpy as np
import torch
import Setup
from Setup import pipeline
import pyrealsense2 as rs
# if Setup.JETSON_MODE:

import multiprocessing

global diameterofwheel 
diameterofwheel = 65

import threading



class SensingInput:
    def __init__(self,ser, GPS=0, IMU=0, INTELLISENSECAMERA=0, V2VLISTENER=0, BNOLISTENER=0):
        self.camera_resolutionx = Setup.camera_resolutionx
        self.camera_resolutiony = Setup.camera_resolutiony

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
        #try:
            print("Intellisense"+str(pipeline))
            frames = pipeline.wait_for_frames()
            self.depth_image = frames.get_depth_frame()
            self.colorframeraw = frames.get_color_frame()

            self.timestamp = self.depth_image.get_timestamp()
    #print(self.colorframeraw)
            self.colorframe = np.asanyarray(self.colorframeraw.get_data())
            # Reset the counter

            # Get the color frame


            # Convert the depth frame to a NumPy array



            # Convert the color frame to a NumPy array

            self.accel = frames[2].as_motion_frame().get_motion_data()

            self.gyro = frames[3].as_motion_frame().get_motion_data()
            #print(self.gyro)
            #print(self.accel)
            # Display the depth image


       # except:
            self.errorhandle.append(1) ## Returns 1 if intellisense fails to capture frame
            return 0
        #finally:
            return 1
            # Wait for a key press

            # Exit the loop if the 'q' key is pressed

    def worker(num):
        """thread worker function"""
        print('Worker: %s' % num)
        return

    def velocity(self):
           try:
            # Read a line of data from the serial port
            data = self.ser.readline().decode().strip()

            #print(data)
	
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
		
 #           print("ERROR IN RPM")
            return "ERROR IN RPM"
    def gettilt(self):

        # Get the current tilt angle
         ##tilt_angle = depth_sensor.get_option(rs.option.camera_angle)

        #print("Tilt angle: ", tilt_angle)
        ##  self.tilt = tilt_angle ## FAILURE
        return 0 ## FAILURE


    def senseall(self):  # runs through all methods to refresh the senses. ## returns bit to sendto debug layer
        #try:
            self.Intellsensor()

            self.velocity()
            return  1
        #except:
            return 0

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

    def Nazrulsobjectdetection(self):
        colorframe = self.colorframe
        depthframe = self.depth_image
        image_data = image_preprocess(np.copy(colorframe), [input_size, input_size])
        image_data = image_data[np.newaxis, ...].astype(np.float32)
        pred_bbox = Yolo.predict(image_data)
        pred_bbox = [tf.reshape(x, (-1, tf.shape(x)[-1])) for x in pred_bbox]
        pred_bbox = tf.concat(pred_bbox, axis=0)
        bboxes = postprocess_boxes(pred_bbox, original_image, input_size, score_threshold)
        bboxes = nms(bboxes, iou_threshold, method='nms')

        return bboxes, image_data



