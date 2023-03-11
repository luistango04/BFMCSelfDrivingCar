import sys
import Setup
from matplotlib import pyplot as plt, cm, colors

from lanedetection import *
from sign_detection_roboflow_rs import *
sys.path.append('.')

import cv2
import numpy as np


import time




class PScene:
    def __init__(self, SensingInput = None):
        self.camera_resolutionx = Setup.camera_resolutionx
        self.camera_resolutiony = Setup.camera_resolutiony

        if(SensingInput is not None):

            self.colorframe = SensingInput.colorframe
            print("FRAME IS NOT NONE")
            #cv2.imshow("COLOR", self.colorframe)
        else:
            sampleframe = cv2.imread(r"D:\BOSCH MOBILITY\BFMCSELFDRIVINGCAR\reallofscenter.png")
            self.frame = sampleframe

        ##    self.intersection_detection = np.zeros((camera_resolution[0], camera_resolution[1]))
        ##    self.midlane = np.zeros((camera_resolution[0], camera_resolution[1]))
        self.sign_trigger = False
        self.intersection_trigger = False
        self.traffic_light_trigger = False
        self.position = 0
        self.SensingInput = SensingInput
        self.objecttrigger = False
        self.position = 0
        self.deviation = 0
        self.direction= 0

	

    def runobjectdetection(self,frame):



        ## NEED THIS TO RUN
        ## do something to make   SensingInput.colorframe look liek the region of interest .
        # c1 = ((int)(.2 * camera_resolutionx), (int)(.3 * camera_resolutiony))  ## TOP LEFT
        # c2 = [0, (int)(.7 * camera_resolutiony)]  ## BOTTOM LEFT
        # c3 = [camera_resolutionx, (int)(.7 * camera_resolutiony)]  ## BOTTOM RIGHT
        # c4 = [(int)(.8 * camera_resolutionx), (int)(.3 * camera_resolutiony)]  # TOP RIGHT
        # ##
        #
        # src = np.float32([c1, c2, c3, c4])
        #
        # # Window to be shown ## NEED ADJUSTMENT  WHEN GO LIVE TO HANDLE THE RESOLUTIONS
        # p1 = [0, 0]  ## TOP LEFT
        # p2 = [0, .7* camera_resolutiony]  ## BOTTOM LEFT
        # p3 = [.8*resoltuion, .7*camera_resolutiony]  ## BOTTOM RIGHT
        # p4 = [.8* camera_resolutionx, 0]  # TOP RIGHT
        #
        # dst = np.float32([p1, p2, p3, p4])
        #
        # # Matrix to warp the image for birdseye window
        # matrix = cv2.getPerspectiveTransform(src, dst)
        #RUN HISTORGRAM LANEDTECTION ON MATRX
        run(matrix,resolutionx,resolutiony)
        ## add additional param for resolution x, resolution y
        ## in case you need it for the tensor function param

    def intersectiondetection(self):


                frameintersect = self.colorframe
                camera_resolutionx = self.camera_resolutionx
                camera_resolutiony = self.camera_resolutiony
                start_time = time.time()

                # Read the input image
                bottomroi = (.9 * camera_resolutiony)
                c1 = ((int)(.4 * camera_resolutionx), (int)(.5 * camera_resolutiony))  ## TOP LEFT

                c2 = ((int)(.45 * camera_resolutionx), (int)(bottomroi))  ## BOTTOM LEFT

                c3 = ((int)(.55 * camera_resolutionx), (int)(bottomroi))  ## BOTTOM RIGHT
                c4 = ((int)(.6 * camera_resolutionx), (int)(.5 * camera_resolutiony))  # TOP RIGHT

                ## OVER COMPENSATE
                # Window to be shown ## NEED ADJUSTMENT  WHEN GO LIVE TO HANDLE THE RESOLUTIONS
                p1 = [0, 0]  ## TOP LEFT
                p2 = [(.001 * camera_resolutionx), camera_resolutiony]  ## BOTTOM LEFT
                p3 = [(.999 * camera_resolutionx), camera_resolutiony]  ## BOTTOM RIGHT
                p4 = [camera_resolutionx, 0]  # TOP RIGHT
                # p5 = [camera_resolutionx/2,0]  # TOP RIGHT
                dst = np.float32([p1, p2, p3, p4])
                src = np.float32([c1, c2, c3, c4])
                ############################# new test code to look for intersection lane
                ## Step 1 Rotate Frame
                birdView = perspectiveWarpintersect(frameintersect, src, dst, camera_resolutionx, camera_resolutiony)
                rotated = cv2.rotate(birdView, cv2.ROTATE_90_CLOCKWISE)
                #cv2.imshow("Rotated", rotated)
                # cv2.waitKey(15000)

                img, hls, grayscale, thresh, blur, canny = processImage(rotated)
                hist, midpoint, highest_peak_x, average_location, highest_peak_y, revertthis, dataline = plotHistogramintersection(
                    thresh)
                # plt.clf()
                # plt.plot(hist)
                # plt.show()
                print(highest_peak_y)
                if (highest_peak_y > 40000):
                    ##print xlocation and message that intersection has been found.
                    print("intersectionfound")
                    print("Location:" + str(highest_peak_x) + " above birds eye view need to calibrate")
                self.intersection_trigger = True
                return bottomroi + bottomroi
                ## REVERT THIS IS THE LOCATION OF INTERSECTION. Just needs calibration. The rest of the qualifying

                # #  THIS CODE IS FOR VISUALIZING RESULTS BETTER ON DEBUGrotatedback = cv2.line(rotated, tuple(dataline[:, 0]), tuple(dataline[:, 1]), (0, 0, 255), 3)

                # rotatedback = cv2.rotate(rotatedback, cv2.ROTATE_90_COUNTERCLOCKWISE)
                #
                # test = perspectiveWarpintersect(rotatedback, dst, src, camera_resolutionx, camera_resolutiony)
                # #  THIS CODE IS FOR VISUALIZING RESULTS BETTER ON DEBUG
                # base_img = frameintersect.copy()
                #
                # # Loop through all the pixels in the second image
                # for i in range(test.shape[0]):
                #     for j in range(test.shape[1]):
                #         # Check if the pixel is colored (i.e., not grayscale)
                #         if not all(test[i, j] == test[i, j][0]):
                #             # Add the pixel to the corresponding pixel in the base image
                #             base_img[i, j] = test[i, j]
                #
                # # Display the resulting image
                # cv2.imshow('Overlay', base_img)

    def lane_detection(self):
        # try:
            c1 = ((int)(.2 * self.camera_resolutionx), (int)(.3 *  self.camera_resolutiony))  ## TOP LEFT
            c2 = [0, (int)(.7 *  self.camera_resolutiony)]  ## BOTTOM LEFT
            c3 = [ self.camera_resolutionx, (int)(.7 *  self.camera_resolutiony)]  ## BOTTOM RIGHT
            c4 = [(int)(.8 *  self.camera_resolutionx), (int)(.3 *  self.camera_resolutiony)]  # TOP RIGHT

            src = np.float32([c1, c2, c3, c4])
            ## OVER COMPENSATE
            # Window to be shown ## NEED ADJUSTMENT  WHEN GO LIVE TO HANDLE THE RESOLUTIONS


            ## OVER COMPENSATE
            # Window to be shown ## NEED ADJUSTMENT  WHEN GO LIVE TO HANDLE THE RESOLUTIONS
            p1 = [0, 0]  ## TOP LEFT
            p2 = [(.001 * self.camera_resolutionx), self.camera_resolutiony]  ## BOTTOM LEFT
            p3 = [(.999 * self.camera_resolutionx), self.camera_resolutiony]  ## BOTTOM RIGHT
            p4 = [self.camera_resolutionx, 0]  # TOP RIGHT
            # p5 = [camera_resolutionx/2,0]  # TOP RIGHT
            dst = np.float32([p1, p2, p3, p4])
        #cv2.imshow

            birdView, birdViewL, birdViewR, minverse = perspectiveWarp(self.colorframe,src,dst,self.camera_resolutionx,self.camera_resolutiony)


            #cv2.imshow("birdView", birdView)
            img, hls, grayscale, thresh, blur, canny = processImage(birdView)
            imgL, hlsL, grayscaleL, threshL, blurL, cannyL = processImage(birdViewL)
            imgR, hlsR, grayscaleR, threshR, blurR, cannyR = processImage(birdViewR)
            histogram, leftxBase, rightxBase,midpoint = plotHistogram(thresh)
            #plt.plot(histogram)
            #plt.show()

#
            ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, histogram)
            draw_info = general_search(thresh, left_fit, right_fit)
            #curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx)
            
            #     #
            #     #
            #     # # Filling the area of detected lanes with green
            meanPts, result = draw_lane_lines(self.colorframe, thresh, minverse, draw_info)
            deviation,direction = offCenter(meanPts,self.colorframe)
#            deviation = -1*pixelDeviation * Setup.xm_per_pix
#            direction = "left" if deviation < 0 else "right"
            
            #curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx,Setup.ym_per_pix,Setup.xm_per_pix) ## IF curavture is needed
            #meanPts, result = draw_lane_lines(self.frame, thresh, minverse, draw_info)
            # print(deviation)
            # print(direction)

   
            #cv2.imshow('birdViewR', hlsR)
            #cv2.imshow('birdViewL', hlsL)
            #print(draw_info)
            self.deviation = deviation
            self.direction = direction
            return deviation,direction

            #ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, hist)


            return img, hls, grayscale, thresh, blur, canny

        # except Exception as e:
            elapsed_time = time.time() - Setup.starttime
            print("Error occurred at time: {:.2f} seconds".format(elapsed_time))
            print("Error message:", e)

    def sign_detection(self):
        sign_index_matrix = infer()
        return sign_index_matrix



#### END - LOOP TO PLAY THE INPUT IMAGE ########################################
################################################################################

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
        return 'midlane: {}\ndeviation:{}\ndirection:{}\nsign_trigger: {}\nintersection_trigger: {}\ntraffic_light_trigger: {}\nposition: {}\n'.format(
            0,
            self.sign_trigger,self.deviation,self.direction ,self.intersection_trigger, self.traffic_light_trigger, self.position)

# ################################################################################
# #### START - FUNCTION TO APPLY PERSPECTIVE WARP ################################
#
# pipeline = Setup.init()
#
# Sense = SensingInput(ser,pipeline)
#
# # Create a config and configure the pipeline to stream from the bag file
#
#
# time.sleep(2)
