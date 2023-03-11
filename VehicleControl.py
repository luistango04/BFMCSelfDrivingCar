



class vehiclecontrol:

    def __init__(self, brain,ser, Sensinginput = None):
        self.brain = brain
        self.Sensinginput = Sensinginput
        self.prev_error = 0
        self.steering = 0
        self.velorate = 0
        self.ser = ser
        self.accelrate = 0
        self.preverrors = (0,0)
        self.steeringcommands  = []
        self.accelerationcommands = []
        self.steeringcap = [-23,23]

    def updatefrombrainscene(self, Brain,PScene):
        self.brain = Brain
        self.Sensinginput = PScene
        self.control()



    def control(self):
        # Check each instance variable and perform actions
        if self.brain.break_trigger:
            self.break_execution()
        elif self.brain.road_search:
            #self.brain.perform_road_search()
            pass
        elif self.brain.switch_lane:
            self.perform_lane_switch()
        elif self.brain.parking:
            self.perform_parking()
        elif self.brain.lane_follow:
            self.lanefollow()
        elif self.brain.acceleration:
            self.accel()
        elif self.brain.intersection:
            self.intersectionmanagement()
        else:
            self.brain.default_action()



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
        self.steeringcommands = [(0, 0, 0)]
        self.accelerationcommands = [(0, 0, 0)]
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



    def intersectionmanagement(self):
        trigger = self.brain.intersection
        if trigger == 'left':
            self.turn_left()
            self.steeringcommands = [(0,0,2),(-15,.5,1),(0,1.5,0)]
            self.velorate = 0.3
            self.current_direction = 'left'

        elif trigger == 'right':
            self.turn_right()
            self.current_direction = 'right'
        elif trigger == 'straight':
            self.go_straight()
            self.current_direction = 'straight'
        else:
            print(f"Invalid trigger: {trigger}")
    def turn_left(self):
        print("Turning left")

    def turn_right(self):
        print("Turning right")

    def go_straight(self):
        print("Going straight")

    def lanefollow(self):
        ## Flush serial
        self.ser.flush()

        # stay and correct to center of Lane
        Kp = 1 # Proportional gain
        Kd = 0.2  # Derivative gain

        # Define initial error and derivative of error
        error = self.brain.lane_follow
        prev_error = 0

        # PD controller
        # while True:
        # Update error and derivative of error

        error_diff = error - self.prev_error
        self.prev_error = error
        print(error_diff)
        # Calculate steering angle
        angle = Kp * error + Kd * error_diff

        # Apply steering angle to the vehicle

        self.steering = angle
        # speed is lastspeed
        return angle

        # return to cruising speed

        # Execute parking function based on trigger value

    def __str__(self):
        return f"UNDERCONSTRUCTION"
        # f"break_signal: {self.break_signal}" \
        # f"\nroad_search_signal: {self.road_search_signal}\nswitch_lane_signal: {self.switch_lane_signal}" \
        # f"\nparking_signal: {self.parking_signal}\nlane_following_signal: {self.lane_following_signal}" \
        # f"\nacceleration_signal: {self.acceleration_signal}\nintersection_navigation_signal: {self.intersection_navigation_signal}"






