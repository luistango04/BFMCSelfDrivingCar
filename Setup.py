import time
import torch
global DEBUG_MODE
global JETSON_MODE
global NAZRUL_MODE
global SERIALDEBUG
global pipeline
import numpy as np
DEBUG_MODE = True
JETSON_MODE = True
NAZRUL_MODE = False
SERIALDEBUG = False
BFMC_MQTT_CONTROL_TOPIC = "bfmc/control"



if JETSON_MODE:
    import pyrealsense2 as rs

    camera_resolutionx = 424
    camera_resolutiony = 240
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, camera_resolutionx, camera_resolutiony, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, camera_resolutionx, camera_resolutiony, rs.format.bgr8, 30)

    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

    profile = pipeline.start(config)
    # get the start time metadata from the camera
    start_time = profile.get_device().get_info(rs.camera_info.recommended_firmware_version)

    time.sleep(2)

if NAZRUL_MODE:
    from yolov3.configs import *
    from yolov3.yolov4 import *
    from yolov3.helper_functions import load_yolo_weights
    import tensorflow as tf
    global YOLO #YOLOv3


def init(ser,DEBUG_MODE = False):

    global camera_resolutionx
    global camera_resolutiony
    global starttime
    global yolo
    starttime = time.time()

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(device)
    model = []
    # Load the model from a .pt file
    #model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # or yolov5n - yolov5x6, custom


    # Results
    global xm_per_pix
    global ym_per_pix
    #Measured distance of bottom part of FOV is 435mm
    # xm = 720/43,5 = 16,55
    kp = 0.1
    ki = 0.03
    kd = 0.005
    k_t = 0.3
    # Defining variables to hold meter-to-pixel conversion
    ym_per_pix = 280 / camera_resolutiony#  ## GUESSING ITS ABOUT THIS FAR Standard lane width is 3.7 cm divided by lane width in 		pixels which is NEEDS TUNING
    # calculated to be approximately 720 pixels not to be confused with frame height
    #xm_per_pix = 35  # camera_resolutionx
    xm_per_pix = 0.82547 # camera_resolutionx,55
    starttime = time.time()  ## PROGRAM START
    ## RUN THIS TO DO SET SENSOR


    if NAZRUL_MODE:
        yolo = Load_Yolo_model()
    pidcarsetting(kp,ki,kd,k_t,ser)
    time.sleep(1)
    print("Setup Done", pipeline)
    return  model,start_time


## put planned activities connection protocol here : to set up and establish connection
# def setupmqtt():



def pidcarsetting(kp,ki,kd,k_t,ser):
    # kp proportional time
    # ki integral coefficient
    # kd derivativecoefficient
    # k_t integraltime
    # ser serial handler

    # write srial fucntions to Car
    ser.write(b'#2:0;;\r\n')
    ser.write(b'#4:1;;\r\n')
    command = f"#6:{kp};{ki};{kd};{k_t};;\r\n".encode()
    ser.write(command)
    ser.write(b'#5:1;;\r\n')
    ser.readline()

    # parse read line validate system settings.....

    # hand the system until validation

    # after 10 seconds throw exception and rebooot

    # read serial  back to car
    return 1

    def __str__(self):
        return f"yaw_rate: {self.yaw_rate}, lateral_acceleration: {self.lateral_acceleration}, longitudinal_acceleration: {self.longitudinal_acceleration}, speed: {self.speed}, steering_wheel_angle: {self.steering_wheel_angle}, steering_wheel_: {self.steering_wheel_velocity}"


def Load_Yolo_model():
    gpus = tf.config.experimental.list_physical_devices('GPU')
    if len(gpus) > 0:
        print(f'GPUs {gpus}')
        try:
            tf.config.experimental.set_memory_growth(gpus[0], True)
        except RuntimeError:
            pass

    if YOLO_FRAMEWORK == "tf":  # TensorFlow detection
        if YOLO_TYPE == "yolov4":
            Darknet_weights = YOLO_V4_TINY_WEIGHTS if TRAIN_YOLO_TINY else YOLO_V4_WEIGHTS
        if YOLO_TYPE == "yolov3":
            Darknet_weights = YOLO_V3_TINY_WEIGHTS if TRAIN_YOLO_TINY else YOLO_V3_WEIGHTS

        if YOLO_CUSTOM_WEIGHTS == False:
            print("Loading Darknet_weights from:", Darknet_weights)
            yolo = Create_Yolo(input_size=YOLO_INPUT_SIZE, CLASSES=YOLO_COCO_CLASSES)
            load_yolo_weights(yolo, Darknet_weights)  # use Darknet weights
        else:
            checkpoint = f"./checkpoints/{TRAIN_MODEL_NAME}"
            if TRAIN_YOLO_TINY:
                checkpoint += "_Tiny"
            print("Loading custom weights from:", checkpoint)
            yolo = Create_Yolo(input_size=YOLO_INPUT_SIZE, CLASSES=TRAIN_CLASSES)
            yolo.load_weights(checkpoint)  # use custom weights

    elif YOLO_FRAMEWORK == "trt":  # TensorRT detection
        saved_model_loaded = tf.saved_model.load(YOLO_CUSTOM_WEIGHTS, tags=[tag_constants.SERVING])
        signature_keys = list(saved_model_loaded.signatures.keys())
        yolo = saved_model_loaded.signatures['serving_default']

    return yolo

    # write srial fucntions to Car


    # read serial  back to car
    return 1

    def __str__(self):
        return f"yaw_rate: {self.yaw_rate}, lateral_acceleration: {self.lateral_acceleration}, longitudinal_acceleration: {self.longitudinal_acceleration}, speed: {self.speed}, steering_wheel_angle: {self.steering_wheel_angle}, steering_wheel_: {self.steering_wheel_velocity}"
def _generate_dummy_pipeline():
    class DummyPipeline:
        def start(self, config):
            pass

        def wait_for_frames(self):
            depth_frame = rs.depth_frame(width=640, height=480)
            color_frame = rs.video_frame(width=640, height=480, format=rs.format.bgr8)
            accel_frame = rs.motion_frame([1, 2, 3])
            gyro_frame = rs.motion_frame([4, 5, 6])
            return rs.composite_frame([depth_frame, color_frame, accel_frame, gyro_frame])

        def stop(self):
            pass

    return DummyPipeline()


#shared_value = mp.Value('f', 0.0)
#
# # Create a camera thread and pass the shared value as an argument
# camera_thread = CameraThread()
#
# # Start the camera thread
# camera_thread.start()
# colorframe,depthframe = camera_thread.get_latest_frame()
# # Read the shared value from outside the thread
# while True:
#     cv2.imshow('color', colorframe)
#     cv2.waitKey(1)
def Intellsensor():  ## captures frame stores in class  # returns 1 if success 0 if fail
    try:
        print("Intellisense"+str(pipeline))
        frames = pipeline.wait_for_frames()

        # get the accelerometer frame
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        # Convert the depth frame to a NumPy array



        # Convert the color frame to a NumPy array

        #accel = frames[2].as_motion_frame().get_motion_data()

        #gyro = frames[3].as_motion_frame().get_motion_data()
        #print(self.gyro)
        #print(self.accel)
        # Display the depth image


    except:
        errorhandle.append(1) ## Returns 1 if intellisense fails to capture frame
        return 0
    #finally:
        return 1
        # Wait for a key press

