



class vehiclecontrol:

    def __init__(self, brain,ser, Sensinginput = None):
        self.brain = brain
        self.Sensinginput = Sensinginput
        self.prev_error = 0

        self.ser = ser
        self.accelrate = 0
        self.preverrors = (0,0)
        self.steeringcommands  = []
        self.velocommands = []
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
            self.default_action()


    def default_action(self):
        # Do something when no trigger is activated
        # ...
        # send velocity to very slow and steering to center
        self.velocommands = [(.18,0, 0)]
        self.steeringcommands = [(0,0, 0)]
        pass

    def flush(self):
        self.ser.flush()



    def break_execution(self):
        # Get break trigger value from brain object
        self.breaktrigger = True
        self.steeringcommands = [(0, 0, 0)]
        self.velocommands = [(0, 0, 0)]
        ## FLUSH SERIAL SEND 0 TO CASH
        self.ser.flush()


        self.velorate = 0
        self.acceleration = 100
        self.steering = 0

        #break_trigger = self.brain.break_trigger

        # Execute break function based on trigger value
    def accel(self,rate = .1):

          self.steeringcommands = [(0, 0, 0)]
          self.velocommands = [(self.accelrate, 0, 0)]
          print("Accelerating")

    def intersectionmanagement(self):
        trigger = self.brain.intersection
        if trigger == 'left':
            self.turn_left()

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
        self.steeringcommands = [(0, 0, 2), (-18, 2, 1), (0, 7, 0)]
        self.velocommands = [(.2, 0, 0)]
        print("Turning left")

    def turn_right(self):
        self.steeringcommands = [(0, 0, 2), (18, 2, 1), (0, 5, 0)]
        self.velorate = 0.3
        print("Turning left")

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
        self.steeringcommands = [(angle, 0, 0)]

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






