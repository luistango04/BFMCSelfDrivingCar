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


class Actuation:
    def __init__(self,VehicleControl):
        self.VehicleControl = VehicleControl
        self.steering = self.VehicleControl.get_steering()

        self.velocity = self.VehicleControl.get_velorate() * maxspeed

        print("VELO:" + str(self.velocity))
        self.acceleration = self.VehicleControl.acc2()
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

        carspeed = self.velocity

        step = time.time() - lasttime
        print(step)
        print ('##')
        print(carspeed)
        print('##')
        print(self.acceleration)


        # If the current speed is less than the target velocity, increase the speed
        if (carspeed < self.velocity):
            carspeed = min(velocity, carspeed + (self.acceleration * step))  ## DAMPENINING

        # If the current speed is greater than the target velocity, decrease the speed
        elif (carspeed > self.velocity):
            carspeed = max(self.velocity, carspeed - (self.acceleration * step))

        # Encode the command string and write it to the serial port
        command = f"#1:{carspeed};;\r\n".encode()
        print("Current Speed:" + str(round(carspeed)) + "  target =" + str(
            round(self.velocity)) + "  seconds  elapsed :" + str(time.time() - starttime))
        print("PRINTED: " + str(command) + " To console")
        ser.write(command)

        # If the current speed is equal to the target velocity, return 1
        if (carspeed == self.velocity):
            return 1, time.time()

        # Otherwise, return 0 to indicate that the speed is still accelerating or decelerating
        return 0, time.time()

    def get_steering(self):
        return self.steering



    def __str__(self):
        return " Input Steering: {} | Velocity: {}".format(self.steering, self.velocity, )

