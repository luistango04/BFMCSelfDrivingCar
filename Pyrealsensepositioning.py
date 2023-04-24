#import torch
import Setup
from Setup import DEBUG_MODE, JETSON_MODE, NAZRUL_MODE, SERIALDEBUG,pipeline
from Sense import SensingInput
from unittest.mock import Mock
from SCENE import PScene
import time
from Brain import Brain
from MQTTGenericClient import MQTTGenericClient
from GenericJsonReader import GenericJsonReader
from VehicleControl import vehiclecontrol
from Actuation import bothfree, velofree, steeringfree
import Actuation
import cv2
import serial

##########
if NAZRUL_MODE:
	from yolov3.helper_functions import det_obj_est_dis
	from yolov3.configs import *
##########



# Dont forget to turn on the fan sudo sh -c "echo 255 > /sys/devices/pwm-fan/target_pwm"

#jsonReader = GenericJsonReader("MQTTVehicleControlMessages.json")
#mqttControlMessage = MQTTGenericClient("jetsonCar", 1, jsonReader)
#mqttControlMessage.start_client()F
#mqttControlMessage.subscribe(Setup.BFMC_MQTT_CONTROL_TOPIC)

if (SERIALDEBUG):
    ser = Mock()
else:
    try:
        ser = serial.Serial('/dev/ttyACM0', 19200, timeout=0.1)
    except:
        ser = serial.Serial('/dev/ttyACM1', 19200, timeout=0.1)

model,start_time = Setup.init(ser)
Sense = SensingInput(ser)
ser.flush()


#
time.sleep(1)  # Give time to fire up camera birghtness
Scene = PScene(Sense)

Brain = Brain(Scene)
vehiclecontrol = vehiclecontrol(Brain, ser, Sense)
Act = Actuation.Act(vehiclecontrol, ser)

start_time = time.time()


import pyrealsense2 as rs
import numpy as np
import time
import math
from vpython import *
import numpy as np
import matplotlib.pyplot as plt



class KalmanFilter:
    def __init__(self, A, B, H, Q, R, P, x):
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P
        self.x = x

    def predict(self, dt, u):
        # Update A and B matrices with new dt value
        self.A[0, 1] = dt
        self.B[1, 0] = dt

        # Perform prediction step
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, z):
        # Perform update step
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.eye(self.P.shape[0]) - np.dot(K, self.H)), self.P)


# Initialize Kalman filter
# Example usage in a loop with changing dt value
x = np.array([[0], [0], [0]])  # Initial state vector
P = np.diag([1000, 1000, 1000])  # Initial covariance matrix
A = np.array([[1, 0, 0.5], [0, 1, 0], [0, 0, 1]])  # State transition matrix
B = np.array([[0.5], [1], [0]])  # Control matrix
H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # Observation matrix
Q = np.diag([0.1, 0.1, 0.1])  # Process noise covariance matrix
R = np.diag([0.1, 0.1, 0.1])  # Measurement noise covariance matrix

kf = KalmanFilter(A=A, B=B, H=H, Q=Q, R=R, P=P, x=x)
kv = KalmanFilter(A=A, B=B, H=H, Q=Q, R=R, P=P, x=x)
kp = KalmanFilter(A=A, B=B, H=H, Q=Q, R=R, P=P, x=x)







# loop to retrieve accelerometer data
while True:
    # wait for a new frame

    Sense.senseall()



    # get the accelerometer data as a numpy array
    #gettretrival timestamp

    timestamp = Sense.timestamp/1000

    # get the gyro data as a numpy array

    accel_data = Sense.accel
    gyro_data = Sense.gyro
    # combine two arrays

    #combine two arrays
    acceleration = np.array([timestamp,accel_data.x, accel_data.y, accel_data.z])- [0,adjustmentfactor[0],adjustmentfactor[1],adjustmentfactor[2]]


    gyroacceleration = np.array([timestamp,gyro_data.x, gyro_data.y, gyro_data.z])- [0,adjustmentfactor[3],adjustmentfactor[4],adjustmentfactor[5]]
    combinedacceleration = np.array([timestamp,accel_data.x, accel_data.y, accel_data.z,gyro_data.x, gyro_data.y, gyro_data.z])- [0,adjustmentfactor[0],adjustmentfactor[1],adjustmentfactor[2],adjustmentfactor[3],adjustmentfactor[4],adjustmentfactor[5]]


    dt = timestamp - lastacceldata[0]
    u = np.array([[dt]])  # Control input vector
    z = np.array([[acceleration[1], acceleration[2], acceleration[3]]])  # Measurement vector

    kf.predict(dt=dt, u=u)
    kf.update(z=z)
    #print(kf.x)
    #combinedacceleration[1:7] = kf.x.flatten()
    print(f"Prediction: {kf.x.flatten()}, dt={dt}")


    print(x)
    #print the data
    #print(data)
    # Print time stamp
    print("Timestamp: {}".format(timestamp))
    ## integrate pitch roll yaw
    gyrovelocity,gyroposition = integrate_accel(gyroacceleration,lastgyrodata,gyrovelocity,gyroposition)

    accelerationinworldplane = transform_acceleration(combinedacceleration,gyroposition)

    accelerationinworldplane[0] = timestamp


    #velocity, position = integrate_accel(accelerationinworldplane, lastacceldata, velocity, position,kv,kp)



    #

    # velocity,position = integrate_accel(acceleration, lastacceldata,velocity,position)
    #print("Velocity values (m/s): ({:.3f}, {:.3f}, {:.3f})".format(velocity[1], velocity[2], velocity[3]))
    # print("Position values (m): ({:.3f}, {:.3f}, {:.3f})".format(position[1], position[2], position[3]))
    update_plot(np.array([timestamp,accel_data.x, accel_data.y, accel_data.z]),velocity, position,fig,axes,Sense.velo)
    #
    lastgyrodata = gyroacceleration
    lastacceldata = accelerationinworldplane
