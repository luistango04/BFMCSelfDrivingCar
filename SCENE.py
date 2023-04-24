import numpy as np
from scipy.stats import norm
import sys
import Setup
from matplotlib import pyplot as plt, cm, colors
from Setup import DEBUG_MODE
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
            #print("FRAME IS NOT NONE")
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
        self.traffic_light_object_trigger = False
        self.crossing_trigger = False
        self.stop_trigger = False
        self.parking_trigger = False
        self.priority_trigger = False
        self.signs = []
        self.distancetocar = 0

        self.nolane = False




    def makeascene(self,model):
        # try:
        #     self.runobjectdetection()
        # except requests.exceptions.ConnectionError as e:
        #     print("Failed to establish a connection:", e)
        # except requests.exceptions.Timeout as e:
        #     print("Request timed out:", e)
        # except requests.exceptions.RequestException as e:
        #     print("An error occurred:", e)
        try:
            self.intersectiondetection()
        except:
            print("ERROR IN Intersection ")
        try:
            self.deviation,self.direction = (self.lane_detection())

        except:
            print("lanedetectionfailure")

        try:
            self.distancetocarf(model)
        except:
            print("distance to car failure")

    def runobjectdetection(self):

            # Wait for a coherent pair of frames: depth and color
            # frames = pipeline.wait_for_frames()
            # color_frame = frames.get_color_frame()
            #
            # # Convert images to numpy arrays
            # color_image = np.asanyarray(color_frame.get_data())
            #
            # # Resize (while maintaining the aspect ratio) to improve speed and save bandwidth
            # height, width, channels = color_image.shape
            color_image  = self.colorframe
            scale = ROBOFLOW_SIZE / max(camera_resolutiony, camera_resolutionx)  ##
            img = cv2.resize(color_image, (round(scale * camera_resolutionx), round(scale * camera_resolutiony)))
            self.signs = infer(img)
            sign_index_matrix = infer(img)
            # update Boolean variables based on the classes that are required right now.

            if 0 in sign_index_matrix:
                self.crossing_trigger = True
            else:
              self.crossing_trigger = False

            if 5 in sign_index_matrix:
                self.parking_trigger = True
            else:
               self.parking_trigger = False

            if 6 in sign_index_matrix:
               self.priority_trigger = True
            else:
               self.priority_trigger = False

            if 7 in sign_index_matrix:
              self.stop_trigger = True
            else:
              self.stop_trigger = False

            if 8 in sign_index_matrix:
                self.traffic_light_object_trigger = True
            else:
                self.traffic_light_object_trigger = False

             ## traffic sign? 8
             ## traffic stop sign 7
                     ## pedestrian cross 0
                     ## parking 5
                     ## priority 6
            return 1

    def distancetocarf(self,model):
        results = cardistance(model, self.SensingInput.colorframeraw)
        if(results is None):
            return 0

        center_x = int(results[0][0])
        center_y = int(results[0][1])

        self.distancetocar =float(100*(self.SensingInput.depth_image.get_distance(center_x,center_y)))
        print("Distance to Car:"+ str(self.distancetocar))
        return self.distancetocar
    def intersectiondetection(self):


                frameintersect = self.colorframe
                camera_resolutionx = self.camera_resolutionx
                camera_resolutiony = self.camera_resolutiony
                start_time = time.time()

                bottomroi = (.95 * camera_resolutiony)

                toproi = (.65 * camera_resolutiony)
                c1 = ((int)(.3 * camera_resolutionx), (int)(toproi))  ## TOP LEFT

                c2 = ((int)(.1 * camera_resolutionx), (int)(bottomroi))  ## BOTTOM LEFT

                c3 = ((int)(.9 * camera_resolutionx), (int)(bottomroi))  ## BOTTOM RIGHT
                c4 = ((int)(.6 * camera_resolutionx), (int)(toproi))  # TOP RIGHT

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
                #cv2.waitKey(5000)

                img, hls, grayscale, thresh, blur, canny = processImage(rotated)
                hist, midpoint, highest_peak_x, average_location, highest_peak_y, revertthis, dataline = plotHistogramintersection(
                    thresh)
                #plt.clf()
                #plt.plot(hist)
                #plt.show()

                #print(highest_peak_y)
                if (highest_peak_y > 46000):
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
            # Read the input image
            bottomroi = (1 * camera_resolutiony)

            toproi = (.6 * camera_resolutiony)
            c1 = ((int)(.3 * camera_resolutionx), (int)(toproi))  ## TOP LEFT

            c2 = ((int)(.1 * camera_resolutionx), (int)(bottomroi))  ## BOTTOM LEFT

            c3 = ((int)(.9 * camera_resolutionx), (int)(bottomroi))  ## BOTTOM RIGHT
            c4 = ((int)(.7 * camera_resolutionx), (int)(toproi))  # TOP RIGHT

            src = np.float32([c1, c2, c3, c4])
                ## OVER COMPENSATE
                # Window to be shown ## NEED ADJUSTMENT  WHEN GO LIVE TO HANDLE THE RESOLUTIONS


            ## OVER COMPENSATE
            # Window to be shown ## NEED ADJUSTMENT  WHEN GO LIVE TO HANDLE THE RESOLUTIONS
            p1 = [-.3*self.camera_resolutionx, 0]  ## TOP LEFT
            p2 = [(.001 * self.camera_resolutionx), self.camera_resolutiony]  ## BOTTOM LEFT
            p3 = [(.999 * self.camera_resolutionx), self.camera_resolutiony]  ## BOTTOM RIGHT
            p4 = [1.3*self.camera_resolutionx, 0]  # TOP RIGHT
            # p5 = [camera_resolutionx/2,0]  # TOP RIGHT
            dst = np.float32([p1, p2, p3, p4])

            if(DEBUG_MODE):
                frame = self.colorframe
            # Get image size

                #img_size = (inpImage.shape[1], inpImage.shape[0])
                # print(img_size)
                # Perspective points to be warped
                ############ update this to identify region lane of interest based on lens of camera

                # Matrix to warp the image for birdseye window

                #cv2.circle(frame, c1, 5, (0, 0, 255), -1)
                #cv2.circle(frame, c2, 5, (0, 0, 255), -1)
                #cv2.circle(frame, c3, 5, (0, 0, 255), -1)
                #cv2.circle(frame, c4, 5, (0, 0, 255), -1)
                #cv2.imshow("ROIS", frame)

    #cv2.imshow
            try:
                birdView, birdViewL, birdViewR, minverse = perspectiveWarp(self.colorframe,src,dst,self.camera_resolutionx,self.camera_resolutiony)

                if(DEBUG_MODE):
                    cv2.imshow("birdView", birdView)
                    cv2.imshow("birdViewL", birdViewL)
                    cv2.imshow("birdViewR", birdViewR)
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
                #print(meanPts)
                deviation,direction = offCenter(meanPts,self.colorframe)
                if(DEBUG_MODE):
                    cv2.imshow("RESULTS:", result)
                    print(direction)
                    print(deviation)


                #curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx,Setup.ym_per_pix,Setup.xm_per_pix) ## IF curavture is needed
                #meanPts, result = draw_lane_lines(self.frame, thresh, minverse, draw_info)
                # print(deviation)
                # print(direction)


                #cv2.imshow('birdViewR', hlsR)
                #cv2.imshow('birdViewL', hlsL)
                #print(draw_info)
                #self.deviation = deviation
                #self.direction = direction
                return deviation, direction
            except:
                deviation = 0
                direction = "straight"
                return deviation, direction
                self.nolane = True
                print("No lanes found")


            #ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, hist)


            return img, hls, grayscale, thresh, blur, canny

        # except Exception as e:
            elapsed_time = time.time() - Setup.starttime
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
def cardistance(model,current_frame):
    current_frame = np.asanyarray(current_frame.get_data())

    #cv2.imshow("camera",current_frame)
    if(NAZRUL_MODE):
        results = model(current_frame)
        colorframe = current_frame

        image_data = image_preprocess(np.copy(colorframe), [input_size, input_size])
        image_data = image_data[np.newaxis, ...].astype(np.float32)
        pred_bbox = Yolo.predict(image_data)

    else:
        results = model(current_frame)
    #print(results)
    # Extract the bounding box coordinates, class indices, and scores for each detected object

    bboxes = results.xyxy[0]
    class_indices = results.pred[0][:, -1].long().tolist()
    scores = results.pred[0][:, -1].tolist()


    # Iterate through each bounding box and get its four corners, xmid, ymid, score, and type
    objects_list = []

    for i, bbox in enumerate(bboxes):
        # Get the score and type of the object
        score = scores[i]
        obj_type = class_indices[i]

        # Process only objects with a type of 2 or 3
        if obj_type in [2, 3]:
            # Extract the top-left and bottom-right coordinates of the bounding box
            x1, y1, x2, y2 = bbox[:4]

            # Calculate the four corners of the bounding box
            top_left = (int(x1), int(y1))
            top_right = (int(x2), int(y1))
            bottom_left = (int(x1), int(y2))
            bottom_right = (int(x2), int(y2))

            # Calculate the midpoint of the bounding box
            xmid = int((x1 + x2) / 2)
            ymid = int((y1 + y2) / 2)
            # Calculate the four corners of the bounding box
            top_left = (int(x1), int(y1))
            bottom_right = (int(x2), int(y2))

            # Draw the bounding box on the frame
            cv2.rectangle(current_frame, top_left, bottom_right, (0, 255, 0), 2)
            # Add text box above the bounding box




            # Add the object's information to the list of objects

            mu = current_frame.shape[1] / 2  # Mean of the distribution (middle of the frame)


            sigma = current_frame.shape[1] / 6  # Standard deviation of the distribution (1/6th of the frame width)
            x_distribution = norm(mu, sigma).pdf(xmid)

            # Multiply the midpoint by the score and the x distribution
            xmid_scored = xmid * score * x_distribution
            #print("exdistribution is : " + str(xmid_scored))
            # Add text box above the bounding box
            text = f"{xmid_scored:.2f}"  # Format the score as a string with 2 decimal places
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)

            text_pos = (int(x1), int(y1 - text_size[1]))  # Position the text box above the bounding box

            cv2.rectangle(current_frame, text_pos, (text_pos[0] + text_size[0], text_pos[1] + text_size[1]),
                          (0, 255, 0), -1)
            cv2.putText(current_frame, text, text_pos, font, font_scale, (0, 0, 0), thickness)
            # Add text box above the bounding box
            objects_list.append({
                'corners': (top_left, top_right, bottom_left, bottom_right),
                'xmid': xmid,
                'ymid': ymid,
                'score': score,
                'type': obj_type,
                'xmid_scored': xmid_scored

            })

    #cv2.imshow("camera", current_frame)
    #key = cv2.waitKey(1)
    #if key == 27:
     #   return
    #time.sleep(3)
    if(objects_list == []):
        return None
    else:
        highest_score_object = max(objects_list, key=lambda obj: obj['score'])
        results = [[highest_score_object['xmid'], highest_score_object['ymid']], [0], [highest_score_object['xmid_scored']]]
        # Return the list of objects

        return results

    def Nazrulsobjectdetection(self):
        colorframe = self.colorframe

        image_data = image_preprocess(np.copy(colorframe), [input_size, input_size])
        image_data = image_data[np.newaxis, ...].astype(np.float32)
        pred_bbox = Yolo.predict(image_data)
        pred_bbox = [tf.reshape(x, (-1, tf.shape(x)[-1])) for x in pred_bbox]
        pred_bbox = tf.concat(pred_bbox, axis=0)
        bboxes = postprocess_boxes(pred_bbox, original_image, input_size, score_threshold)
        bboxes = nms(bboxes, iou_threshold, method='nms')

        return bboxes, image_data


