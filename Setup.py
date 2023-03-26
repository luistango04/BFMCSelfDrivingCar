import time
import cv2
import numpy as np

global DEBUG_MODE
global JETSON_MODE
global NAZRUL_MODE
DEBUG_MODE = True
JETSON_MODE = True
NAZRUL_MODE = False

if JETSON_MODE:
    import pyrealsense2 as rs
    #import usb.core  # Import for the USB library
    #import usb.core  # Import for the USB library
    #import usb.util  # Import for the USB library

def init(ser,DEBUG_MODE = False):
    pipeline = []
    global camera_resolutionx
    global camera_resolutiony
    global starttime
    global yolo
    starttime = time.time()
    camera_resolutionx = 424
    camera_resolutiony = 240
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

    if JETSON_MODE:
        pipeline = camerainit(camera_resolutionx, camera_resolutiony)
    if NAZRUL_MODE:
        yolo = Load_Yolo_model()
    pidcarsetting(kp,ki,kd,k_t,ser)
    time.sleep(1)
    return pipeline


## put planned activities connection protocol here : to set up and establish connection
# def setupmqtt():

def camerainit(camera_resolutionx, camera_resolutiony):

#    Part to reset/reattach camera connection through software
  #  Find the device
  #   try:
  #       dev = usb.core.find(idVendor=0x8086, idProduct=0x0b3a) #Intel D435i
  #
  #       #If the device is found, reset its USB connection
  #       if dev is not None:
  #           try:
  #               #Detach the device from the kernel driver
  #               if dev.is_kernel_driver_active(0):
  #                   dev.detach_kernel_driver(0)
  #
  #               #Reset the device
  #               dev.reset()
  #
  #               #Reattach the device to the kernel driver
  #               usb.util.dispose_resources(dev)
  #               dev.attach_kernel_driver(0)
  #
  #           #If there is an error, print it
  #           except usb.core.USBError as e:
  #               print("USBError: {}".format(str(e)))
  #
  #       #If the device is not found, print an error message
  #       else:
  #           print("USB device not found")
  #   except:
  #       if DEBUG_MODE:
  #           return _generate_dummy_pipeline()
    time.sleep(2)



    # Configure depth and color streams
    pipeline = rs.pipeline()

    # Configure the pipeline to stream both color, depth and motion
    config = rs.config()
    config.enable_stream(rs.stream.depth, camera_resolutionx, camera_resolutiony, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, camera_resolutionx, camera_resolutiony, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)


    # Display the color image using OpenCV

   
    #    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    # Start streaming
    pipeline.start(config)
    return pipeline

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
