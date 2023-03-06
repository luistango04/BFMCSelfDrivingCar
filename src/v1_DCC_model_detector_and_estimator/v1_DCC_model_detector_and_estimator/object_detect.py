import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'
import cv2
import numpy as np
import tensorflow as tf
from yolov3.helper_functions import det_obj_est_dis, Load_Yolo_model
from yolov3.configs import *
import pyrealsense2
from realsense_depth import *
import time

dc = DepthCamera()

yolo = Load_Yolo_model()

fps_counter = 0
start = time.time()

while True:
    ret, depth_frame, color_frame = dc.get_frame()

    image = det_obj_est_dis(yolo, color_frame, "./IMAGES/kite_pred.jpg", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0), depth_frame=depth_frame)
    
    end = time.time()
    fps_counter = fps_counter + 1
    if (end - start) >= 1:
        print("FPS : ", fps_counter)
        fps_counter = 0
        start = time.time()

    cv2.imshow("Color frame", image)
    key = cv2.waitKey(1)
    if key == 27:
        break