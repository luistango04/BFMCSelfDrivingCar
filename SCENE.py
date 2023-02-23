import sys
import serial
from Sense import SensingInput
from  Actuation import Actuation
from  VehicleControl import VehicleControl
from unittest.mock import Mock
#from Sign_detection_yolo import detect
import time
sys.path.append('.')
import cv2
import matplotlib.pyplot as plt
import numpy as np

class PScene:
    def __init__(self, SensingInput = [], camera_resolutionx = 420, camera_resolutiony = 320):
        self.camera_resolution = camera_resolutionx
        self.camera_resolution = camera_resolutiony
        #SensingInput.colorframe
        #SensingInput.colorframe
        ##    self.intersection_detection = np.zeros((camera_resolution[0], camera_resolution[1]))
        ##    self.midlane = np.zeros((camera_resolution[0], camera_resolution[1]))
        self.sign_trigger = False
        self.intersection_trigger = False
        self.traffic_light_trigger = False
        self.position = 0
        self.SensingInput = SensingInput
        self.objecttrigger = False

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
    def get_object_trigger(self):
        return self.objecttrigger

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

