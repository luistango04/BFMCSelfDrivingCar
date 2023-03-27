



class vehiclecontrol:

    def __init__(self, brain,ser, Sensinginput = None):
        self.brain = brain
        self.Sensinginput = Sensinginput
        self.prev_error = 0

        self.ser = ser
        self.preverrors = (0,0)
        self.steeringcommands  = []
        self.velocommands = []
        self.steeringcap = [-23,23]

    def updatefrombrainscene(self, Brain,PScene):
        self.brain = Brain
        self.Sensinginput = PScene
        self.control()


    def cruisecontrol(self):
        variablehandler = self.brain.distancetocar - self.brain.target

        curspeed = self.Sensinginput.velo
       ## what

        self.steeringcommands = []
        self.velocommands = [(variablehandler, 0, 0)]
        lanefollow()

        pass

    def control(self):
        # Check each instance variable and perform actions
        if self.brain.state == 'OBJECT_AND_SIGN_TRIGGER':
            self.break_execution()
        elif self.brain.state == 'OBJECT_TRIGGER':
            #self.brain.perform_road_search()
            pass
        elif self.brain.state == 'SIGN_TRIGGER':
            self.perform_lane_switch()
        elif self.brain.state == 'INTERSECTION_TRIGGER':
            self.perform_parking()
        elif self.brain.state == 'INTERSECTION AND STOP SIGN':
            print("lane follow Triggered")

            self.lanefollow()

        elif self.brain.state == 'TRAFFIC_LIGHT_TRIGGER':
            self.accel()
        elif self.brain.state == 'CRUISECONTROL':
            self.cruisecontrol()
        elif self.brain.state == 'OBJECT_AND_INTERSECTION_TRIGGER':
            self.cruisecontrol()
        elif self.brain.state == 'NO_TRIGGER':
            print("Intersection Triggered")
            self.intersectionmanagement()
        else:
            self.default_action()


    def default_action(self):
        # Do something when no trigger is activated
        # ...
        # send velocity to very slow and steering to center
        self.ser.flush()
        self.velocommands = [(.3,0, 0)] #velocity commands are (velocity, time, mode)
        self.steeringcommands = [(0,0, 0)] #steering commands are (angle, time, mode)
        pass

    def flush(self):
        self.ser.flush()



    def break_execution(self):
        
        ## FLUSH SERIAL SEND 0 TO CASH
        self.ser.flush()
        # Get break trigger value from brain object
        self.breaktrigger = True
        self.steeringcommands = [(0, 0, 0)]
        self.velocommands = [(0, 0, 0)]

        self.velorate = 0
        self.steering = 0

        #break_trigger = self.brain.break_trigger

        # Execute break function based on trigger value


    def intersectionmanagement(self):
        self.ser.flush()
        trigger = self.brain.intersection
        
        if trigger == 'left':
            self.turn_left()

            self.current_direction = 'left'
        elif trigger == 'stopstraight':
            self.stopstraight()
            self.current_direction = 'stopstraight'
        elif trigger == 'right':
            self.turn_right()
            self.current_direction = 'right'
        elif trigger == 'straight':
            self.go_straight()
            self.current_direction = 'straight'
        else:
            print(f"Invalid trigger: {trigger}")
    def stopstraight(self):
        self.steeringcommands = [(0, 0, 0)]
        self.velocommands = [(0.0, 0, 2),(0.3, 2, 1),(0.3,4,0)]
        print("Turning left")

    def turn_left(self):
        self.steeringcommands = [(0, 0, 2), (-23, 2, 1), (0, 6.5, 0)]
        self.velocommands = [(.3, 0, 0)]
        print("Turning left")

    def turn_right(self):
        self.steeringcommands = [(0, 0, 2), (23, 0.5, 1), (0, 4.5, 0)] #+3 for steeringadjustment of -3 deg
        self.velorate = 0.3
        print("Turning right")

    def go_straight(self):
        print("Going straight")

    def lanefollow(self):
        ## Flush serial
        print("IN LANE CORRECTION")
        self.ser.flush()
        error = self.brain.lane_follow/10
        # stay and correct to center of Lane
        Kp = 0.1 # Proportional gain
        Kd = 0.2  # Derivative gain

        # Define initial error and derivative of error

        prev_error = 0

        # PD controller
        # while True:
        # Update error and derivative of error

        error_diff = error - self.prev_error
        self.prev_error = error
        print(error_diff)
        # Calculate steering angle
        angle =Kp * error + Kd * error_diff
        print(angle)
        # Apply steering angle to the vehicle
        self.steeringcommands = [(angle, 0, 1),(0, .25,0 )]
        self.velocommands = [(.3, 0, 0)]
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






