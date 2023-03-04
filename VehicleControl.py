



class VehicleControl:

    def __init__(self, brain, pscene, vehicle_data,ser):
        self.brain = brain
        self.pscene = pscene
        self.vehicle_data = vehicle_data
        self.steering = 0
        self.velorate = 0
        self.ser = ser
        self.accelrate = 0
        self.preverrors = (0,0)
        self.actuationchoreography  = []


    def __call__(self, steering, velorrate, accelrate,ser):

        self.steering = steering
        self.velorate = velorrate
        self.ser = ser
        self.accelrate = accelrate

        print("Instance is called via special method")
        return self

    def flush(self):
        self.ser.flush()

        return self.steering


    def get_steering(self):
        return self.steering

    def acc2(self):
        return self.accelrate
    def get_velorate(self):
        return self.velorate

    def break_execution(self):
        # Get break trigger value from brain object
        self.breaktrigger = True
        ## FLUSH SERIAL SEND 0 TO CASH
        self.ser.flush()


        self.velorate = 0
        self.acceleration = 100
        self.steering = 0

        #break_trigger = self.brain.break_trigger

        # Execute break function based on trigger value
    def accel(self,rate = .75):
        # Get break trigger value from brain object
        self.velorate =  rate ## % of max velocity ## DEFAULT VELOCITY
        self.acceleration = 5 ## % increase per step time
        ## FLUSH SERIAL SEND 0 TO CASH





    def lanefollow(self, ):
        ## Flush serial
        self.ser.flush()
        # stay and correct to center of Lane
        Kp = 0.1  # Proportional gain
        Kd = 0.01  # Derivative gain

        # Define initial error and derivative of error
        error = xcenter_lane - xcenter_image
        prev_error = 0

        # PD controller
        # while True:
        # Update error and derivative of error

        error_diff = error - self.prev_error
        prev_error = error

        # Calculate steering angle
        angle = Kp * error + Kd * error_diff

        # Limit the steering angle to the maximum and minimum values
        angle = max(min_angle, min(max_angle, angle))
        print(angle)
        # Apply steering angle to the vehicle
        self.steering = angle
        # speed is lastspeed
        return angle, VEHICLE.speed

        # return to cruising speed

        # Execute parking function based on trigger value

    def __str__(self):
        return f"UNDERCONSTRUCTION"
        # f"break_signal: {self.break_signal}" \
        # f"\nroad_search_signal: {self.road_search_signal}\nswitch_lane_signal: {self.switch_lane_signal}" \
        # f"\nparking_signal: {self.parking_signal}\nlane_following_signal: {self.lane_following_signal}" \
        # f"\nacceleration_signal: {self.acceleration_signal}\nintersection_navigation_signal: {self.intersection_navigation_signal}"






