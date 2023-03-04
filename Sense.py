import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyrealsense2 as rs
import Setup



global diameterofwheel 
diameterofwheel = 65

class SensingInput:
    def __init__(self,ser,pipeline, GPS=0, IMU=0, INTELLISENSECAMERA=0, V2VLISTENER=0, BNOLISTENER=0):
        self.pipeline = pipeline ##
        self.speed = 0
        self.colorframe = []
        self.ser = ser
        self.accel = 0
        self.gyro = 0
        self.velo = 0
        self.tilt = 0

        self.imu =  0   ## 0 is no imu 1 is imu

    def Intellsensor(self):  ## captures frame stores in class  # returns 1 if success 0 if fail
        # try:

        ##try:

            # Wait for frames
            frames =self.pipeline.wait_for_frames()


            color_frame = frames.get_color_frame()

            # Convert the color frame to a NumPy array
            color_array = np.asanyarray(color_frame.get_data())
                    # Get the depth frame
            depth_frame = frames.get_depth_frame()

            # self.gyro = gyro_data(f[3].as_motion_frame().get_motion_data())
            self.colorframe = frames.get_color_frame()

            self.color_array = np.asanyarray(self.color_frame.get_data())
            cv2.imshow("Color Image", self.colorframe)
            print(self.colorframe)
            self.depth = np.asanyarray(frames.get_depth_frame().get_data())
            # Get the color frame
            color_frame = frames.get_color_frame()

            np.set_printoptions(threshold=np.inf)

            # for y in range(480):
            #
            #     for x in range(640):
            #         dist = self.depth.get_distance(x, y)
            #         if 0 < dist and dist < 1:
            #             coverage[x//10] += 1

            # print(coverage)
            # with open ('depth.txt', 'w') as f:
            #     f.write(str(self.depth))
            #     f.close()
            #
            # cv2.imwrite('depth.jpg', self.depth)
            # Convert the depth frame to a NumPy array

            # Convert the color frame to a NumPy array


            # Convert the color image from BGR to RGB

            # Display the depth image
            #cv2.imshow("Depth Image", depth_image)

            # Display the color image
            #cv2.imshow("Color Image", color_image)

            # Wait for a key press
            key = cv2.waitKey(1)

            # Exit the loop if the 'q' key is pressed


        ##finally:


            #self.accel = accel_data(f[2].as_motion_frame().get_motion_data())

            #self.tilt = f[1].as_motion_frame().get_motion_data().z

            #print("accelerometer: ", self.accel)
            #print("gyro: ", self.gyro)




            return 1


    # def getimu(self):
    #     try:
    #         ### DO THS JOB to get the IMU ALUES
    #     return [x,y,z,etc]
    #     except:
    #         print("IMU NOT FOUND")
    #         return 0
    def velocity(self):
           try:
            # Read a line of data from the serial port
            data = self.ser.readline().decode().strip()

            print(data)
	
            if data:
                # Parse the data to extract the RPM value
                if data.startswith("@5:"):
                    rpm = float(data.split(":")[1].split(";")[0])
      
                    self.velo = rpm * diameterofwheel ## Diameter of wheel
                    print(rpm)
 				
                    return self.velo 
            else:
               print('No data from serial')
               self.velo = 0
               return self.velo 

           except:
		
            print("ERROR IN RPM")
            return 0
    def gettilt(self):

        # Get the current tilt angle
         ##tilt_angle = depth_sensor.get_option(rs.option.camera_angle)

        #print("Tilt angle: ", tilt_angle)
        ##  self.tilt = tilt_angle ## FAILURE
        return 0 ## FAILURE


    def senseall(self):  # runs through all methods to refresh the senses. ## returns bit to sendto debug layer
     	
        self.velocity()

        results = self.Intellsensor()

        return self.tilt,self.velo

    def get_COLORFRAME(self):


        return self.colorframe

    def get_GPS(self):
        return self.GPS

    def get_IMU(self):
        return self.IMU

    def get_INTELLISENSECAMERA(self):
        return self.INTELLISENSECAMERA

    def get_V2VLISTENER(self):
        return self.V2VLISTENER

    def get_BNOLISTENER(self):
        return self.BNOLISTENER

    def get_ROADIMAGE(self):
        return self.ROADIMAGE

    def get_SIGNREGION(self):
        return self.SIGNREGION

    def get_INTERSECTIONREGION(self):
        return self.INTERSECTIONREGION

    def get_TRAFFICSTATE(self):
        return self.TRAFFICSTATE

    def get_CAR_POS(self):
        return self.CAR_POS

    def get_BNO_POS(self):
        return self.BNO_POS

def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])


