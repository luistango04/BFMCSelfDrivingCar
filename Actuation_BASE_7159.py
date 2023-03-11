import time


#### CAR GLOBAL MAX
global maxsteering
maxsteering = 23
global minsteering
minsteering = -23
global maxspeed
maxspeed = .3
global minspeed
minspeed = -.3
global lastangle 
lastangle = 0




class Actuation:
    def __init__(self,VehicleControl,ser):
        self.commandlist = VehicleControl.actuationchoreography
        self.steeringangle = VehicleControl.get_steering()
        self.angularacceleration = 10
        self.velocity = VehicleControl.get_velorate() * maxspeed
        self.ser = ser

        #print("VELO:" + str(self.velocity))
        self.acceleration = VehicleControl.acc2()

        #print("Accelo:" + str(self.acceleration))
    def update(self,VehicleControl):
        self.commandlist = VehicleControl.actuationchoreography
        self.steeringangle = VehicleControl.get_steering()
        self.angularacceleration = 10
        self.velocity = VehicleControl.get_velorate() * maxspeed


        #print("VELO:" + str(self.velocity))
        self.acceleration = VehicleControl.acc2()

        #print("Accelo:" + str(self.acceleration))
    def write_velocity_command(self, ser,lasttime,starttime):
        """
        This function writes a velocity command to the given serial port `ser` with the specified `velocity` and `acceleration`.

        Parameters:
            velocity (float): The target velocity to be set.
            acceleration (float): The acceleration of the device in M/S.
            ser (serial.Serial): The serial port to write the command to.
            VEHICLE.speed (float): The last recorded speed of the device.

        Returns:
            Tuple: A tuple containing the following values:
                int:
                    1 if the speed has reached the target velocity,
                    0 if the speed is still accelerating or decelerating,
                    -1 if an error occurred.
                float: The current speed of the device.
                float: The time elapsed since the start of the function.
        """
        # Calculate the time step since the last update

        # Initialize the current speed with the last recorded speed#

        carspeed = self.carspeed

        step = time.time() - lasttime
        # print(step)
        # print ('##')
        # print(carspeed)
        # print('##')
        # print(self.acceleration  * step)

        print(carspeed)
        print(self.velocity)
        # If the current speed is less than the target velocity, increase the speed
        if (carspeed < self.velocity):
            carspeed = min(self.velocity, max(.15,carspeed + (self.acceleration * step)))  ## DAMPENINING  #15 withot PID .1PID

        # If the current speed is greater than the target velocity, decrease the speed
        elif (carspeed > self.velocity):
            carspeed = max(self.velocity, carspeed - 4*(self.acceleration * step))
        elif (round(carspeed,6) > round(self.velocity,6)):
            carspeed = self.velocity

        # Encode the command string and write it to the serial port
        command = f"#1:{carspeed};;\r\n".encode()
        print("Current Speed:" + str(round(carspeed,8)) + "  target =" + str(self.velocity) + "  seconds  elapsed :" + str(time.time() - starttime))
        print("PRINTED: " + str(command) + " To console")
        self.ser.write(command)

        # If the current speed is equal to the target velocity, return 1
        if (carspeed == self.velocity):
            return 1, time.time(),carspeed

        # Otherwise, return 0 to indicate that the speed is still accelerating or decelerating
        return 0, time.time(),carspeed

    def write_angle_command(self):
        """
        This function writes a steering command to the given serial port `ser` with the specified `steeringangle` and `angularacceleration`.

        Parameters:
            steering (float): The target angle to be set. Maximum bounds of angle is -23.0 to 23.0
            acceleration (float): The angularacceleration of the device in deg/S*S.
            ser (serial.Serial): The serial port to write the command to.
            lastangle (float): The last recorded anle of the device.


        Returns:
            Tuple: A tuple containing the following values:
                int:
                    1 if the steeringangle has reached the target angle,
                    0 if the steeringangle is still increasing or decreasing,
                    -1 if an error occurred.
                float: The current angle of the steering.
                float: The time elapsed since the start of the function.
        """
        self.steeringangle = 1*check_angle(self.steeringangle )
	

        command = f"#2:{round(self.steeringangle,5)};;\r\n".encode()

        print("PRINTED: " + str(command) + " To console")

        self.ser.write(command)

        # If the current angle is equal to the target steering angle, return 1

        # Otherwise, return 0 to indicate that the angle is still increasing or decreasing
        return 0, time.time(),self.steeringangle
    def get_steering(self):
        return self.steering



    def __str__(self):
        return " Input Steering: {} | Velocity: {}".format(self.steering, self.velocity, )
def check_angle(angle):
    if angle > 23:
        return 23
    elif angle < -23:
        return -23
    else:
        return angle
