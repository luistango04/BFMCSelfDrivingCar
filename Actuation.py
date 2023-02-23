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
    def __init__(self,VehicleControl,carspeed):

        self.steering = VehicleControl.get_steering()
        self.angularacceleration = 10
        self.carspeed = carspeed
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
        ser.write(command)

        # If the current speed is equal to the target velocity, return 1
        if (carspeed == self.velocity):
            return 1, time.time(),carspeed

        # Otherwise, return 0 to indicate that the speed is still accelerating or decelerating
        return 0, time.time(),carspeed

    def write_angle_command(self, ser, lasttime,starttime):
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

        # Calculate the time step since the last update
        step = time.time() - lasttime

        # Initialize the current angle with the last recorded angle
        steeringangle = lastangle

        # If the current angle is less than the target steering, increase the angle
        if (steeringangle < self.steering):
            steeringangle = min(self.steering, steeringangle + (self.angularacceleration * step)) ## DAMPENINING

        # If the current angle is greater than the target steering angle, decrease the angle
        elif (steeringangle > self.steering):
            steeringangle = max(self.steering, steeringangle - (self.angularacceleration * step))

        # Encode the command string and write it to the serial port
        command = f"#2:{steeringangle};;\r\n".encode()
        print("Current Angle:" + str(round(steeringangle)) + "  target =" + str(
            round(self.steering)) + "  seconds  elapsed :" + str(time.time() - starttime))
        print("PRINTED: " + str(command) + " To console")
        ser.write(command)

        # If the current angle is equal to the target steering angle, return 1
        if (steeringangle == self.steering):
            return 1, time.time(), steeringangle

        # Otherwise, return 0 to indicate that the angle is still increasing or decreasing
        return 0, time.time(), steeringangle
    
    def get_steering(self):
        return self.steering



    def __str__(self):
        return " Input Steering: {} | Velocity: {}".format(self.steering, self.velocity, )

