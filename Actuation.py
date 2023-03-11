import time
import threading
import time
import serial



# Create a Serial object for the desired port and configure it with the appropriate settings


# Iterate over the list of commands and start a new thread for each command

# Continue with the rest of the program without waiting for the serial write

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
global steeringadjustment
steeringadjustment = -3
global vehiclefree
vehiclefree = True

# Define a function to perform the serial write operation
def perform_steering_write(command, delay,listitem, ser):
    # Wait for the specified delay
    if(not(listitem)):
        vehiclefree = True

    else:
        vehiclefree = False

    print("DEBUG:" + str(command+steeringadjustment) + " To console" + "Delay:" + str(delay))
    time.sleep(delay)
    # Perform serial write comman
    command = f"#2:{round(command+steeringadjustment, 5)};;\r\n".encode()

    ser.write(command)
def perform_drive_write(command, delay,listitem, ser):
    # Wait for the specified delay
    if(not(listitem)):
        vehiclefree = True

    else:
        vehiclefree = False

    command = f"#1:{carspeed};;\r\n".encode()
    time.sleep(delay)
    # Perform serial write comman
    command = f"#1:{round(command+steeringadjustment, 5)};;\r\n".encode()

    ser.write(command)

class Actuation:
    def __init__(self,VehicleControl,ser):
        self.steeringcommands = VehicleControl.steeringcommands
        self.steeringangle = VehicleControl.get_steering()
        self.angularacceleration = 10
        self.velocity = VehicleControl.get_velorate() * maxspeed
        self.ser = ser
        self.write_thread = None

        # Create a threading.Event object to signal when the serial write thread is in progress
        self.write_thread_in_progress = threading.Event()
        #print("VELO:" + str(self.velocity))
        self.acceleration = VehicleControl.acc2()

        def is_writing(self):
            # Check if the write thread is in progress
            return self.write_thread_in_progress.is_set()
        #print("Accelo:" + str(self.acceleration))
    def update(self,VehicleControl):
        self.steeringcommands = VehicleControl.steeringcommands
        self.steeringangle = VehicleControl.get_steering()
        self.angularacceleration = 10
        self.velocity = VehicleControl.get_velorate() * maxspeed
        self.write_steering_command()
 #       self.write_velocity_command(self.ser,self.lasttime,self.starttime)
        self.write_velocity_command()
        #print("VELO:" + str(self.velocity))
        self.acceleration = VehicleControl.acc2()

        #print("Accelo:" + str(self.acceleration))
    def write_velocity_command(self):
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
        commands = self.steeringcommands
        for command, delay,listitem in commands:
            serial_thread = threading.Thread(target=perform_drive_write, args=(command, delay,listitem, self.ser))
            serial_thread.start()

        # Otherwise, return 0 to indicate that the speed is still accelerating or decelerating
        return 0, time.time()

    def write_steering_command(self):
        """
        This function writes a steering command to the given serial port `ser` with the specified `steeringangle` and `angularacceleration`.


        """
        commands = self.steeringcommands
        for command, delay,listitem in commands:
            serial_thread = threading.Thread(target=perform_steering_write, args=(command, delay,listitem, self.ser))
            serial_thread.start()
            # Store a reference to the current write thread


            # Wait for the serial write thread to finish


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
