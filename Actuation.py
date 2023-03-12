import time
import threading
import time
import serial
from unittest.mock import Mock
from Setup import DEBUG_MODE

# Create a Serial object for the desired port and configure it with the appropriate settings
velofree = True
steeringfree = True
bothfree = True

def update_bothfree():
    global bothfree, velofree, steeringfree
    bothfree = velofree and steeringfree


# Iterate over the list of commands and start a new thread for each command

# Continue with the rest of the program without waiting for the serial write
#### CAR GLOBAL MAX
global maxsteering
maxsteering = 23
global minsteering
minsteering = -23
global maxspeed
maxspeed = 1.0 # 1m/s max speed
global minspeed
minspeed = -0.18 # 0.18m/s min speed
global lastangle 
lastangle = 0
global steeringadjustment
steeringadjustment = -3 # Adjustment for steering misalignment. Positive is right, negative is left



# Define a function to perform the serial write operation
def perform_steering_write(command, delay,listitem, ser):
    # Wait for the specified delay
    expected_time = time.time() + delay
    global steeringfree


    time.sleep(delay)

    actual_time = time.time()

    if(DEBUG_MODE):
        print(f"{time.time():.3f} - Delaying for {delay:.3f} seconds...")
        print(f"{actual_time:.3f} - Expected: {expected_time:.3f} - Actual: {actual_time:.3f}")
    command = check_angle(command+steeringadjustment)
    command = f"#2:{round(command, 5)};;\r\n".encode()
    ser.write(command)

    if not listitem:

        steeringfree = True

    else:
        steeringfree = False
    update_bothfree()

def perform_drive_write(command, delay,listitem, ser):
    # Wait for the specified delay
    global velofree
    expected_time = time.time() + delay

    print(f"{time.time():.3f} - Delaying for {delay:.3f} seconds...")
    time.sleep(delay)

    actual_time = time.time()
    print(f"{actual_time:.3f} - Expected: {expected_time:.3f} - Actual: {actual_time:.3f}")

    command = f"#1:{round(command, 5)};;\r\n".encode()
    ser.write(command)

    if not listitem:
        velofree = True
        update_bothfree()


    else:
        velofree = False
        update_bothfree()
    update_bothfree()
class Actuation:
    def __init__(self,VehicleControl,ser = Mock()):
        self.steeringcommands = VehicleControl.steeringcommands
        self.velocommands = VehicleControl.velocommands
        self.ser = ser
        self.write_thread = None



        def is_writing(self):
            # Check if the write thread is in progress
            return self.write_thread_in_progress.is_set()
        #print("Accelo:" + str(self.acceleration))
    def update(self,VehicleControl):
        self.steeringcommands = VehicleControl.steeringcommands
        self.velocommands = VehicleControl.velocommands
        self.write_steering_command()
 #       self.write_velocity_command(self.ser,self.lasttime,self.starttime)
        self.write_velocity_command()
        #print("VELO:" + str(self.velocity))

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
        commands = self.velocommands
        velofree  = False
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
        steeringfree = False
        for command, delay,listitem in commands:
            serial_thread = threading.Thread(target=perform_steering_write, args=(command, delay,listitem, self.ser))
            serial_thread.start()
            # Store a reference to the current write thread


            # Wait for the serial write thread to finish


        # If the current angle is equal to the target steering angle, return 1

        # Otherwise, return 0 to indicate that the angle is still increasing or decreasing
        return 0, time.time()


    def __str__(self):
        return " Input Steering: {} | Velocity: {}".format(self.steeringcommands, self.velocommands, )
def check_angle(angle):
    if angle > 23:
        return 23
    elif angle < -23:
        return -23
    else:
        return angle



# start_time = time.time()
# steeringcommands = [(0,0,2),(-15,2,1),(0,5,0)]
# commands = steeringcommands
# print(velofree + steeringfree)
# steeringfree = False
# for command, delay, listitem in commands:
#
#     print(command, delay, listitem)
#     serial_thread = threading.Thread(target=perform_steering_write, args=(command, delay, listitem, ser))
#     serial_thread.start()
#
#     # Store a reference to the cu
# print(bothfree)
# time.sleep(2)
# print(velofree + steeringfree)
# time.sleep(2)
# print(velofree + steeringfree)
# time.sleep(2)
# print(velofree + steeringfree)
# time.sleep(2)
# print(velofree + steeringfree)
#
# print(bothfree)
#
