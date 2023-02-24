import sys
from lanedetection import processImage,plotHistogram,slide_window_search,general_search,measure_lane_curvature,draw_lane_lines
#from Sign_detection_yolo import detect
sys.path.append('.')
import matplotlib.pyplot as plt
global camera_resolutionx
global camera_resolutiony
import cv2
import numpy as np
import os
from scipy import optimize
from matplotlib import pyplot as plt, cm, colors


camera_resolutionx = 320
camera_resolutiony = 240
global xm_per_pix
global ym_per_pix
# Defining variables to hold meter-to-pixel conversion
ym_per_pix = 280 / camera_resolutiony#  ## GUESSING ITS ABOUT THIS FAR Standard lane width is 3.7 cm divided by lane width in pixels which is NEEDS TUNING
# calculated to be approximately 720 pixels not to be confused with frame height
xm_per_pix = 35  / camera_resolutionx




class PScene:
    def __init__(self, SensingInput = None):
        self.camera_resolution = camera_resolutionx
        self.camera_resolution = camera_resolutiony
        sampleframe = cv2.imread(r"D:\BOSCH MOBILITY\BFMCSELFDRIVINGCAR\reallofscenter.png")
        if(SensingInput is not None):
            self.frame = SensingInput.get_COLORFRAME()
        else:
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


    def lanenode(self):
        try:
            birdView, birdViewL, birdViewR, minverse = perspectiveWarp(self.frame)
            img, hls, grayscale, thresh, blur, canny = processImage(birdView)
            imgL, hlsL, grayscaleL, threshL, blurL, cannyL = processImage(birdViewL)
            imgR, hlsR, grayscaleR, threshR, blurR, cannyR = processImage(birdViewR)
            histogram, leftxBase, rightxBase,midpoint = plotHistogram(thresh)
            ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, histogram)
            draw_info = general_search(thresh, left_fit, right_fit)

            #     #
            #     #
            #     # # Filling the area of detected lanes with green
            meanPts, result = draw_lane_lines(self.frame, thresh, minverse, draw_info)

            mpts = meanPts[-1][-1][-2].astype(int)
            pixelDeviation = self.frame.shape[1] / 2 - abs(mpts)

            deviation = pixelDeviation * xm_per_pix
            direction = "left" if deviation < 0 else "right"
            ##curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx,ym_per_pix,xm_per_pix) ## IF curavture is needed its available
            # print(deviation)
            # print(direction)
            #cv2.imshow('birdView', hlsR)

            #print(draw_info)
            self.deviation = deviation
            self.direction = direction
            return deviation,direction

            #ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, hist)


            return img, hls, grayscale, thresh, blur, canny

        except Exception as e:
            elapsed_time = time.time() - start_time
            print("Error occurred at time: {:.2f} seconds".format(elapsed_time))
            print("Error message:", e)



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

################################################################################
#### START - FUNCTION TO APPLY PERSPECTIVE WARP ################################
def perspectiveWarp(inpImage):

    # Get image size
    img_size = (inpImage.shape[1], inpImage.shape[0])
    #print(img_size)
    # Perspective points to be warped
    ############ update this to identify region lane of interest based on lens of camera

    c1 = ((int) (.2*camera_resolutionx),(int)(.3*camera_resolutiony)) ## TOP LEFT
    c2 =   [0,(int) (.7*camera_resolutiony)] ## BOTTOM LEFT
    c3 =  [camera_resolutionx, (int)(.7*camera_resolutiony)]   ## BOTTOM RIGHT
    c4 =      [(int) (.8*camera_resolutionx),(int)(.3*camera_resolutiony)] #TOP RIGHT
    ##

    src = np.float32([c1,c2,c3,c4])

    # Window to be shown ## NEED ADJUSTMENT  WHEN GO LIVE TO HANDLE THE RESOLUTIONS
    p1 = [0,0]## TOP LEFT
    p2 = [0,camera_resolutiony]  ## BOTTOM LEFT
    p3 = [camera_resolutionx,camera_resolutiony]  ## BOTTOM RIGHT
    p4 = [camera_resolutionx,0]  # TOP RIGHT

    dst = np.float32([p1,p2,p3,p4])

    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(src, dst)
    #cv2.imshow("myetest2",matrix)
    # cv2.circle(frame,c1,5,(0,0,255),-1)
    # cv2.circle(frame,c2,5,(0,0,255),-1)
    # cv2.circle(frame,c3,5,(0,0,255),-1)
    # cv2.circle(frame,c4,5,(0,0,255),-1)
    # #cv2.imshow("mytest", frame)

    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inpImage, matrix, img_size)

    # Get the birds
    # eye window dimensions
    height, width = birdseye.shape[:2]

    # Divide the birdseye view into 2 halves to separate left & right lanes
    birdseyeLeft  = birdseye[0:height, 0:width // 2]
    birdseyeRight = birdseye[0:height, width // 2:width]

#     Display birdseye view image
    #cv2.imshow("Birdseye" , birdseye)
    #cv2.imshow("Birdseye Left" , birdseyeLeft)
    #cv2.imshow("Birdseye Right", birdseyeRight)

    return birdseye, birdseyeLeft, birdseyeRight, minv
#### END - FUNCTION TO APPLY PERSPECTIVE WARP ##################################
################################################################################


# import time
#
# Scene = PScene()
#
# start_time = time.time()
#
# # Read the input image
#
# #
# # # Display the loaded image
# # #cv2.imshow('image', frame)
# #
# print(Scene.lanenode())
# print(Scene)
#
# #
# # #cv2.imshow('birdView', img)
# #
# # cv2.waitKey(0)
# # cv2.destroyAllWindows()
# start_time = time.time()  # Start timer
# num_iterations = 1000   # Replace with the actual number of iterations in your loop
#
# # Loop code goes here
# for i in range(num_iterations):
#     Scene.lanenode()
#
# end_time = time.time()  # Stop timer
# iterations_per_sec = num_iterations / (end_time - start_time)
#
# print(f"Iterations per second: {iterations_per_sec:.2f}")