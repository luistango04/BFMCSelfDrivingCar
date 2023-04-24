
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
def transform_acceleration(acceleration,gyroposition):
    # Extract the data from the acceleration list
    time, local_x_accel, local_y_accel, local_z_accel, local_roll_accel, local_pitch_accel, local_yaw_accel = acceleration
    time,roll,pitch,yaw = gyroposition


    # Create the rotation matrix
    R_roll = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    R_pitch = np.array(
        [[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    # Multiply the rotation matrices together
    R = np.matmul(R_yaw, np.matmul(R_pitch, R_roll))

    # Create the acceleration vector
    accel_local = np.array([local_x_accel, local_y_accel, local_z_accel, local_roll_accel, local_pitch_accel, local_yaw_accel])

    # Create the transformation matrix
    T = np.zeros((6, 6))
    T[:3, :3] = R
    T[3:, 3:] = np.eye(3)

    # Multiply the transformation matrix by the acceleration vector
    accel_euclidean = np.matmul(T, accel_local)
    print(accel_euclidean)
    print("###")
    return np.array([time, accel_euclidean[0], accel_euclidean[1], accel_euclidean[2], accel_euclidean[3], accel_euclidean[4],
            accel_euclidean[5]])

def init_plot():
    # Initialize plot
    fig, axs = plt.subplots(3, 1)
    axs[1].set_xlabel('acceleration')

    axs[1].set_xlabel('Velocity')
    axs[2].set_xlabel('Position X')


    axs[0].set_xlim([-2, 2])
    axs[0].set_ylim([-2, 2])
    axs[1].set_xlim([-10, 10])
    axs[1].set_ylim([-10, 10])



    plt.grid(True)

    return(fig,axs)


def update_plot(accel,vel, pos,fig,axs,velo):

    # Clear previous points from both axes
    axs[0].cla()
    axs[1].cla()
    axs[2].cla()

    axs[0].plot(0,0, marker='o', markersize=5)

    axs[1].plot(0,0, marker='o', markersize=5)
    axs[2].plot( 0,0, marker='o', markersize=5)

    print(accel)
    #print("Acceleration values (m/s^2): ({:.3f}, {:.3f}, {:.3f})".format(accel[1], accel[2], accel[3]))
    # print("Velocity values (m/s): ({:.3f}, {:.3f}, {:.3f})".format(vel[1], vel[2], vel[3]))
    # print("Position values (m/s): ({:.3f}, {:.3f}, {:.3f})".format(pos[1], pos[2], pos[3]))
    # # Plot velocity and position
    axs[0].plot(accel[1], accel[3], marker='x', markersize=5)
    axs[1].plot(vel[1], vel[3], marker='x', markersize=5)
    #axs[1].plot(velo, 0, marker='x', markersize=8)
    axs[2].plot( pos[1], pos[3], marker='o', markersize=5)

    # Set limits and labels for both axes
    axs[0].set_xlim([-10, 10])
    axs[0].set_ylim([-10, 10])
    axs[1].set_xlim([-10, 10])
    axs[1].set_ylim([-1, 1])
    axs[2].set_xlim([-5, 5])
    axs[2].set_ylim([-5, 5])
    axs[0].set_xlabel('Acceleration')
    axs[1].set_xlabel('Velocity')
    axs[2].set_xlabel('Position X')

    # Plot x over y position


    # Show plot
    # Show plot
    plt.draw()
    plt.pause(0.001)

def integrate_accel(acceleration,lastacceldata, vel, pos, kv = None, kp = None):
    # Extract accelerometer data and time stamps

    oldvel = vel
    oldpos = pos
    dt = acceleration[0]-lastacceldata[0]
    print("times: ", acceleration[0],lastacceldata[0])
    print("Elapsed time: ", dt)

    # Calculate velocity using trapezoidal rule
    vel = vel + 0.5 * (acceleration  + lastacceldata) * dt
    u = np.array([[dt]])  # Control input vector
    z = np.array([[vel[1], vel[2], vel[3]]])  # Measurement vector
    if(kv is not None):
        kv.predict(dt=dt, u=u)
        kv.update(z=z)
        vel[1:3], vel[2], vel[3] = kv.x[0], kv.x[1], kv.x[2]

    print("Final Velocity values (m/s): ({:.3f}, {:.3f}, {:.3f})".format(vel[1], vel[2], vel[3]))


    print("Final velocity: ", vel)
    # Calculate position using trapezoidal rule
    pos = pos + 0.5 * (vel + oldvel) * dt
    if(kp is not None):
        kp.predict(dt=dt, u=u)
        kp.update(z=z)
        pos[1], pos[2], pos[3] = kp.x[0], kp.x[1], kp.x[2]

    return vel, pos


# def update_plot(scene, roll, pitch, yaw):
#     # Convert angles to radians
#     roll_rad = radians(roll)
#     pitch_rad = radians(pitch)
#     yaw_rad = radians(yaw)
#
#     # Calculate direction vectors
#     front = vector(sin(yaw_rad), cos(yaw_rad), 0)
#     right = vector(cos(yaw_rad), -sin(yaw_rad), 0)
#     up = vector(0, 0, 1)
#
#     # Clear existing objects from the scene
#     for obj in scene.objects:
#         obj.visible = False
#
#     # Draw arrows for roll, pitch, and yaw
#     arrow(pos=vector(0, 0, 0), axis=roll * right, color=color.red, shaftwidth=0.1)
#     arrow(pos=vector(0, 0, 0), axis=pitch * front, color=color.green, shaftwidth=0.1)
#     arrow(pos=vector(0, 0, 0), axis=yaw * up, color=color.blue, shaftwidth=0.1)
#

def calc_initial_orientation(data):
    # Calculate total acceleration vector
    rawData_X = data[0]
    rawData_Y = data[1]
    rawData_Z = data[2]
    accelerationX = float(rawData_X)
    accelerationY = float(rawData_Y)
    accelerationZ = float(rawData_Z)
    # print("Acceleration X: ", accelerationX)
    # print("Acceleration Y: ", accelerationY)
    # print("Acceleration Z: ", accelerationZ)

    pitch =  math.atan(accelerationX / math.sqrt(accelerationY * accelerationY + accelerationZ * accelerationZ))
    roll =  math.atan( accelerationY / math.sqrt(accelerationX * accelerationX + accelerationZ * accelerationZ))

    yaw =  math.atan(accelerationZ / math.sqrt(accelerationX * accelerationX + accelerationY * accelerationY))

    print("Pitch: ", pitch)
    print("Roll: ", roll)
    print("Yaw: ", yaw)
    return roll, pitch, yaw

def calibratefactors(Sense):
    data = np.zeros((100,6), dtype=np.float32)


    for i in range(100):
        print(i)

        Sense.senseall()

        # get the accelerometer frame

        # get the accelerometer data as a numpy array
        # gettretrival timestamp
        timestamp = Sense.timestamp/1000

        # get the gyro data as a numpy array

        accel_data = Sense.accel
        gyro_data = Sense.gyro
        # combine two arrays
        data[i] = np.array([ accel_data.x, accel_data.y, accel_data.z, gyro_data.x, gyro_data.y, gyro_data.z])

    # calculate the average values of the accelerometer and gyroscope data
    data = np.mean(data, axis=0)


    return data,timestamp
# create a pipeline and start streaming


# print the start time
