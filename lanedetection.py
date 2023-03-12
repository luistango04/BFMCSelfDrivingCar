
# DESCRIPTION:  THIS  PROJECT WAS CREATED  TO DEMONSTRATE HOW  A  LANE DETECTION
#               SYSTEM WORKS  ON CARS EQUIPPED WITH A FRONT  FACING CAMERA. WITH
#               THE HELP OF OPENCV LIBRARIES IT IS POSSIBLE TO DESIGN ALGORITHMS
#               THAT CAN  IDENTIFY LANE LINES, AND PREDICT STEERING ANGLES, ALSO
#               WARN  DRIVERS  IF THE CAR IS  DRIFTING  AWAY FROM  CURRENT LANE.
################################################################################


import pyrosbag as pb





import cv2
import numpy as np
import os
from scipy import optimize
from matplotlib import pyplot as plt, cm, colors

camera_resolutionx = 424
camera_resolutiony = 240


# Get path to the current working directory
CWD_PATH = os.getcwd()
def getpipelinefrombagfile():
    parser = argparse.ArgumentParser(description='Convert RealSense bag file to stream.')
    parser.add_argument('--input', type=str, required=True, help='input bag file')
    parser.add_argument('--output', type=str, required=True, help='output stream file')
    args = parser.parse_args()

    # Open the bag file for reading
    bag = rsrec.realsense_bag(args.input)

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream from the bag file
    config = rs.config()
    config.enable_device_from_file(args.input)

    # Start the pipeline and get the first frame
    pipeline.start(config)
    return pipeline
############################################################################
#### START - FUNCTION TO APPLY PERSPECTIVE WARP ################################

#### END - FUNCTION TO APPLY PERSPECTIVE WARP ##################################
################################################################################



################################################################################
######## START - FUNCTIONS TO PERFORM IMAGE PROCESSING #########################
################################################################################
#fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
#out = cv2.VideoWriter('output.avi', fourcc, 20.0, (xresolution, yresolution))
################################################################################
#### START - FUNCTION TO READ AN INPUT IMAGE ###################################



def plotHistogramintersection(inpImage):

    histogram = np.sum(inpImage[inpImage.shape[0] // 2:, :], axis = 0)

    midpoint = np.int64(histogram.shape[0] / 2)
    x = np.arange(0, histogram.shape[0], 1)
    plt.xlabel("Image X Coordinates")
    plt.ylabel("Number of White Pixels")
    highest_peak_idx = np.argmax(histogram)
    highest_peak_x = x[highest_peak_idx]

    # Assuming the y-axis values are stored in the variable 'y'
    # Get the y-value at the highest peak index
    highest_peak_y = histogram[highest_peak_idx]

    revertthis =  plt.axvline(x=highest_peak_idx, color='r')

    #get the numpypoints from the plot
    dataline = revertthis.get_data()
    dataline = np.round(dataline).astype(int)
    dataline = np.round(dataline).astype(int)

    # Get the average location of the highest peak
    average_location = (highest_peak_x + highest_peak_y) / 2
    # Return histogram and x-coordinates of left & right lanes to calculate
    # lane width in pixels
   # plt.show()

    return histogram,  midpoint,highest_peak_x, average_location,highest_peak_y,revertthis,dataline
#### END - FUNCTION TO PLOT THE HISTOGRAM OF WARPED IMAGE ######################
################################################################################


def perspectiveWarpintersect(inpImage,src,dst,camera_resolutionx,camera_resolutiony):

    print(type(inpImage))
    # Get image size
#    img_size = (inpImage.shape[1], inpImage.shape[0])
 #   print(type(img_size))
    # Perspective points to be warped
    ############ update this to identify region lane of interest based on lens of camera





    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(src, dst)

    # cv2.imshow("myetest2",matrix)
    # cv2.circle(frame, c1, 5, (0, 0, 255), -1)
    # cv2.circle(frame, c2, 5, (0, 0, 255), -1)
    # cv2.circle(frame, c3, 5, (0, 0, 255), -1)
    # cv2.circle(frame, c4, 5, (0, 0, 255), -1)
    #cv2.imshow("ROIS", frame)

    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inpImage, matrix, (camera_resolutionx,camera_resolutiony))

    # Get the birds
    # eye window dimensions

    return birdseye

def perspectiveWarp(inpImage,src,dst,camera_resolutionx,camera_resolutiony):

    frame = inpImage
    # Get image size
    img_size = (inpImage.shape[1], inpImage.shape[0])
    #print(img_size)
    # Perspective points to be warped
    ############ update this to identify region lane of interest based on lens of camera




    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(src, dst)

    # cv2.imshow("myetest2",matrix)
    # cv2.circle(frame, c1, 5, (0, 0, 255), -1)
    # cv2.circle(frame, c2, 5, (0, 0, 255), -1)
    # cv2.circle(frame, c3, 5, (0, 0, 255), -1)
    # cv2.circle(frame, c4, 5, (0, 0, 255), -1)
    #cv2.imshow("ROIS", frame)

    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inpImage, matrix, img_size)

    # Get the birds
    # eye window dimensions
    height, width = birdseye.shape[:2]

    # Divide the birdseye view into 2 halves to separate left & right lanes
    birdseyeLeft = birdseye[0:height, 0:width // 2]
    birdseyeRight = birdseye[0:height, width // 2:width]

    #     Display birdseye view image

    # cv2.imshow("Birdseye Left" , birdseyeLeft)
    # cv2.imshow("Birdseye Right", birdseyeRight)

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










def readVideo():

    # Read input video from current working directory
    inpImage = cv2.VideoCapture(videopath)

    return inpImage
#### END - FUNCTION TO READ AN INPUT IMAGE #####################################
################################################################################



################################################################################
#### START - FUNCTION TO PROCESS IMAGE #########################################
def processImage(inpImage):

    # Apply HLS color filtering to filter out white lane lines
    hls = cv2.cvtColor(inpImage, cv2.COLOR_BGR2HLS)
    lower_white = np.array([0, 160, 10])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(inpImage, lower_white, upper_white)
    hls_result = cv2.bitwise_and(inpImage, inpImage, mask = mask)

    # Convert image to grayscale, apply threshold, blur & extract edges
    gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh,(3, 3), 0)
    canny = cv2.Canny(blur, 40, 60)

    # Display the processed images
    #cv2.imshow("Image", inpImage)
##    cv2.imshow("HLS Filtered", hls_result)
##    cv2.imshow("Grayscale", gray)
##    cv2.imshow("Thresholded", thresh)
##    cv2.imshow("Blurred", blur)
   #cv2.imshow("Canny Edges", canny)

    return inpImage, hls_result, gray, thresh, blur, canny
#### END - FUNCTION TO PROCESS IMAGE ###########################################
################################################################################




################################################################################
#### START - FUNCTION TO PLOT THE HISTOGRAM OF WARPED IMAGE ####################
def plotHistogram(inpImage):

    histogram = np.sum(inpImage[inpImage.shape[0] // 2:, :], axis = 0)

    midpoint = np.int64(histogram.shape[0] / 2)
    leftxBase = np.argmax(histogram[:midpoint])
    rightxBase = np.argmax(histogram[midpoint:]) + midpoint

    plt.xlabel("Image X Coordinates")
    plt.ylabel("Number of White Pixels")

    # Return histogram and x-coordinates of left & right lanes to calculate
    # lane width in pixels

    return histogram, leftxBase, rightxBase,midpoint
#### END - FUNCTION TO PLOT THE HISTOGRAM OF WARPED IMAGE ######################
################################################################################
#T


################################################################################
#### START - APPLY SLIDING WINDOW METHOD TO DETECT CURVES ######################
def slide_window_search(binary_warped, histogram):

    # Find the start of left and right lane lines using histogram info
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    midpoint = np.int64(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # A total of 9 windows will be used
    nwindows = 9
    window_height = np.int64(binary_warped.shape[0] / nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 100
    minpix = 50
    left_lane_inds = []
    right_lane_inds = []

    #### START - Loop to iterate through windows and search for lane lines #####
    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
        (0,255,0), 2)
        cv2.rectangle(out_img, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
        (0,255,0), 2)
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = np.int64(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int64(np.mean(nonzerox[good_right_inds]))
    #### END - Loop to iterate through windows and search for lane lines #######

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Apply 2nd degree polynomial fit to fit curves


    if len(lefty) > 0 and len(leftx) > 0:
     left_fit = np.polyfit(lefty, leftx, 2)
    else:
     left_fit = [0, 0, 0]
    right_fit = np.polyfit(righty, rightx, 2)

    
    # handle the error here (e.g. set default values for left_fit and right_fit)


    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

    ltx = np.trunc(left_fitx)
    rtx = np.trunc(right_fitx)
    #plt.plot(right_fitx)


    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    plt.imshow(out_img,extent=[0, 424, 0, 240])
    #plt.plot(left_fitx,  ploty, color = 'yellow')
    #plt.plot(right_fitx, ploty, color = 'yellow')
    plt.xlim(0, 1280)
    plt.ylim(720, 0)

    return ploty, left_fit, right_fit, ltx, rtx
#### END - APPLY SLIDING WINDOW METHOD TO DETECT CURVES ########################
################################################################################



################################################################################
#### START - APPLY GENERAL SEARCH METHOD TO DETECT CURVES ######################
def general_search(binary_warped, left_fit, right_fit):

    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy +
    left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) +
    left_fit[1]*nonzeroy + left_fit[2] + margin)))

    right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy +
    right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) +
    right_fit[1]*nonzeroy + right_fit[2] + margin)))
	
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    if len(lefty) > 0 and len(leftx) > 0:
     left_fit = np.polyfit(lefty, leftx, 2)
    else:
     left_fit = [0, 0, 0]
    right_fit = np.polyfit(righty, rightx, 2)

    
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]


    ## VISUALIZATION ###########################################################

    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    window_img = np.zeros_like(out_img)
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin,
                                  ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)


    # plt.plot(left_fitx,  ploty, color = 'yellow')
    # plt.plot(right_fitx, ploty, color = 'yellow')
    plt.xlim(0, 1280)
    plt.ylim(720, 0)


    ret = {}
    ret['leftx'] = leftx
    ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty

    return ret
#### END - APPLY GENERAL SEARCH METHOD TO DETECT CURVES ########################
################################################################################



################################################################################
#### START - FUNCTION TO MEASURE CURVE RADIUS ##################################
def measure_lane_curvature(ploty, leftx, rightx,ym_per_pix,xm_per_pix):

    leftx = leftx[::-1]  # Reverse to match top-to-bottom in y
    rightx = rightx[::-1]  # Reverse to match top-to-bottom in y

    # Choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)

    # Fit new polynomials to x, y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)

    # Calculate the new radii of curvature
    left_curverad  = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in cm
    # print(left_curverad, 'm', right_curverad, 'm')

    # Decide if it is a left or a right curve
    if leftx[0] - leftx[-1] > 60:
        curve_direction = 'Left Curve'
    elif leftx[-1] - leftx[0] > 60:
        curve_direction = 'Right Curve'
    else:
        curve_direction = 'Straight'

    return (left_curverad + right_curverad) / 2.0, curve_direction
#### END - FUNCTION TO MEASURE CURVE RADIUS ####################################
################################################################################



################################################################################
#### START - FUNCTION TO VISUALLY SHOW DETECTED LANES AREA #####################
def draw_lane_lines(original_image, warped_image, Minv, draw_info):

    leftx = draw_info['leftx']
    rightx = draw_info['rightx']
    left_fitx = draw_info['left_fitx']
    right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']

    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    mean_x = np.mean((left_fitx, right_fitx), axis=0)
    pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])
    x_coords = pts_mean[:, :, 0].flatten()
    y_coords = pts_mean[:, :, 1].flatten()


    # Plot the x and y coordinates
    plt.plot(x_coords, y_coords)
    #plt.show()
    # Show the plot

    #print(pts_mean)
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    cv2.fillPoly(color_warp, np.int_([pts_mean]), (0, 255, 255))

    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

    return pts_mean, result
#### END - FUNCTION TO VISUALLY SHOW DETECTED LANES AREA #######################
################################################################################


#### START - FUNCTION TO CALCULATE DEVIATION FROM LANE CENTER ##################
################################################################################

## KEY FUNCTION WILL PASS THE OFFSENT  as WELL AS MED DEVIATION

def offCenter(meanPts, inpFrame):

    # Calculating deviation in cm
    mpts = meanPts[-1][-1][-2].astype(int)
    #print(mpts)
    #print("###")
    pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
    deviation = pixelDeviation
    #print(deviation)
    direction = "right" if deviation < 0 else "left"

    return deviation, direction
################################################################################
#### END - FUNCTION TO CALCULATE DEVIATION FROM LANE CENTER ####################



################################################################################
#### START - FUNCTION TO ADD INFO TEXT TO FINAL IMAGE ##########################
def addText(img, radius, direction, deviation, devDirection):

    # Add the radius and center position to the image
    font = cv2.FONT_HERSHEY_TRIPLEX
    text3 = "Welcome to the Future!"
    text4 = "Greetings from Tallinn"
    if (direction != 'Straight'):
        text = 'Radius of Curvature: ' + '{:04.0f}'.format(radius) + 'm'
        text1 = 'Curve Direction: ' + (direction)

    else:
        text = 'Radius of Curvature: ' + 'N/A'
        text1 = 'Curve Direction: ' + (direction)

    cv2.putText(img, text , (50,80), font, .4, (255,255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, text1, (50,120), font, 0.4,  (255,255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, text3, (50, 160), font, 0.4,  (255,255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, text4, (50, 200), font, 0.4,  (255,255, 255), 1, cv2.LINE_AA)

    # Deviation
    deviation_text = 'Off Center:  ' + str(round(abs(deviation), 3)) + 'm' + ' to the ' + devDirection
    cv2.putText(img, deviation_text, (50, 250), cv2.FONT_HERSHEY_TRIPLEX, 0.4,  (255,255, 255), 1, cv2.LINE_AA)
    # print(text)
    # print(text1)
    # print(text3)
    return img
#### END - FUNCTION TO ADD INFO TEXT TO FINAL IMAGE ############################
################################################################################

################################################################################
######## END - FUNCTIONS TO PERFORM IMAGE PROCESSING ###########################
################################################################################

################################################################################
################################################################################
################################################################################
################################################################################

################################################################################
######## START - MAIN FUNCTION #################################################
################################################################################
import time
from unittest.mock import Mock
import pyrealsense2 as rs
import Setup
import argparse
import Sense
#ser = Mock() ## SET THIS TO SERIAL FOR LIVE!
#ser = serial.Serial('/dev/ttyACM0', 19200, timeout=0.1)

#pipeline = Setup.init()

#Sense = Sense.SensingInput(ser,pipeline)

# Create a config and configure the pipeline to stream from the bag file
#
#
# time.sleep(2)
#
# try:
#     while True:
#         Sense.senseall()
#         # Write the current frame to the stream file
#
#         frameintersect = Sense.get_COLORFRAME()
#
#         start_time = time.time()
#
#         # Read the input image
#
#
#
#         c1 = ((int)(.4 * camera_resolutionx), (int)(.5 * camera_resolutiony))  ## TOP LEFT
#
#         c2 = ((int)(.45* camera_resolutionx), (int)(.9 * camera_resolutiony))  ## BOTTOM LEFT
#
#         c3 = ((int)(.55 * camera_resolutionx), (int)(.9 * camera_resolutiony))  ## BOTTOM RIGHT
#         c4 = ((int)(.6 * camera_resolutionx), (int)(.5 * camera_resolutiony))  # TOP RIGHT
#
#         ## OVER COMPENSATE
#         # Window to be shown ## NEED ADJUSTMENT  WHEN GO LIVE TO HANDLE THE RESOLUTIONS
#         p1 = [0, 0]  ## TOP LEFT
#         p2 = [(.001 * camera_resolutionx), camera_resolutiony]  ## BOTTOM LEFT
#         p3 = [(.999 * camera_resolutionx), camera_resolutiony]  ## BOTTOM RIGHT
#         p4 = [camera_resolutionx, 0]  # TOP RIGHT
#         # p5 = [camera_resolutionx/2,0]  # TOP RIGHT
#         dst = np.float32([p1, p2, p3, p4])
#         src = np.float32([c1, c2, c3, c4])
#         ############################# new test code to look for intersection lane
#         ## Step 1 Rotate Frame
#         birdView = perspectiveWarpintersect(frameintersect,src,dst,camera_resolutionx,camera_resolutiony)
#         rotated = cv2.rotate(birdView, cv2.ROTATE_90_CLOCKWISE)
#         cv2.imshow("Rotated", rotated)
#         cv2.waitKey(15000)
#
#         img, hls, grayscale, thresh, blur, canny = processImage(rotated)
#         hist, midpoint,highest_peak_x, average_location,highest_peak_y,revertthis,dataline = plotHistogramintersection(thresh)
#         if(average_location > 20000):
#             ##print xlocation and message that intersection has been found.
#             print("intersectionfound")
#             print("Location:" + str(highest_peak_x) + " above birds eye view need to calibrate")
#         ## REVERT THIS IS THE LOCATION OF INTERSECTION. Just needs calibration. The rest of the qualifying
#
#
#
#
#         rotatedback = cv2.line(rotated, tuple(dataline[:, 0]), tuple(dataline[:, 1]), (0, 0, 255),3)
#         rotatedback = cv2.rotate(rotatedback, cv2.ROTATE_90_COUNTERCLOCKWISE)
#
#
#
#         test = perspectiveWarpintersect(rotatedback,dst,src,camera_resolutionx,camera_resolutiony)
#
#
#         base_img = frameintersect.copy()
#
#         # Loop through all the pixels in the second image
#         for i in range(test.shape[0]):
#             for j in range(test.shape[1]):
#                 # Check if the pixel is colored (i.e., not grayscale)
#                 if not all(test[i,j] == test[i,j][0]):
#                     # Add the pixel to the corresponding pixel in the base image
#                     base_img[i,j] = test[i,j]
#
#         # Display the resulting image
#         cv2.imshow('Overlay', base_img)
#



        ##plot vertical line



        #cv2.line(img, (250, 0), (250, 499), (255, 0, 0), thickness=2)



        # plt.plot(hist)
        #
        # plt.show()
        # intersectionloc = highest_peak_x
        #
        # text = f"Intersection: ({intersectionloc})"
        # print(text)
        #
        #
        #
        # cv2.imshow("Original", frameintersect)
        # # Show original and rotated images
        # #cv2.imshow("Original2", birdView)
        #
        # cv2.imshow("Returntonormal", base_img)
        # cv2.waitKey(15000)
        #
        #




        # cv2.destroyAllWindows()
# except KeyboardInterrupt:
#     pass
#
#
#
# ################################################################################
######## END - MAIN FUNCTION ###################################################
################################################################################



#
#
#

## Intersection function . Rotate image look for general search
# Display the loaded image
#cv2.imshow('image', frame)

#######################################
#### START - LOOP TO PLAY THE INPUT IMAGE ######################################

#try:


##


# ##FRAME IS CV2 IMage
#
#
# # Apply perspective warping by calling the "perspectiveWarp()" function
# # Then assign it to the variable called (birdView)
# # Provide this function with:
# # 1- an image to apply perspective warping (frame)
# birdView, birdViewL, birdViewR, minverse = perspectiveWarp(frame,c1,c2,c3,c4,camera_resolutionx,camera_resolutiony)
#
#
# # Apply image processing by calling the "processImage()" function
# # Then assign their respective variables (img, hls, grayscale, thresh, blur, canny)
# # Provide this function with:
# # 1- an already perspective warped image to process (birdView)
# img, hls, grayscale, thresh, blur, canny = processImage(birdView)
# imgL, hlsL, grayscaleL, threshL, blurL, cannyL = processImage(birdViewL)
# imgR, hlsR, grayscaleR, threshR, blurR, cannyR = processImage(birdViewR)
#
# # Plot and display the histogram by calling the "get_histogram()" function
# # Provide this function with:
# # 1- an image to calculate histogram on (thresh)
# hist, leftBase, rightBase,midpoint = plotHistogram(thresh)
# # # print(rightBase - leftBase)
# #plt.plot(hist)
#
# #plt.show()
#
# #
#
# #(frame)
# ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, hist)
# plt.plot(left_fit)
# #plt.show()
# #
# #
# draw_info = general_search(thresh, left_fit, right_fit)
# #plt.show()
# #
# #
#
# curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx,17,33)
# #
# #
# # # Filling the area of detected lanes with green
# meanPts, result = draw_lane_lines(frame, thresh, minverse, draw_info)
# #
# #
# deviation, directionDev = offCenter(meanPts, frame)
# print(deviation)
# #
# #
# # # Adding text to our final image
# finalImg = addText(result, curveRad, curveDir, deviation, directionDev)
# plt.show()
# #
# # # Displaying final image
# cv2.imshow("Final", finalImg)
# #      out.write(finalImg)
# #
#

# Wait for the ENTER key to be pressed to stop playback

#except Exception as e:
# elapsed_time = time.time() - start_time
# print("Error occurred at time: {:.2f} seconds".format(elapsed_time))
# print("Error message:", e)

#### END - LOOP TO PLAY THE INPUT IMAGE ########################################
################################################################################

# Cleanup
#out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (xresolution,yresolution))
## intersection ROI
