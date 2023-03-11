from SCENE import PScene

class Brain:

    def __init__(self, PScene):
        if PScene is not None:
            self.update(PScene)

        # Initialize instance variables for the seven triggers
        self.break_trigger = False
        self.road_search = False
        self.switch_lane = False
        self.parking = False
        self.lane_follow = False
        self.acceleration = False
        self.intersection = False
        self.override = False

    def update(self, PScene):
        object_trigger = PScene.get_object_trigger()
        sign_trigger = PScene.get_sign_trigger()
        intersection_trigger = PScene.intersection_trigger
        traffic_light_trigger = PScene.traffic_light_trigger

        state_map = {
            (True, True, False, False): 'OBJECT_AND_SIGN_TRIGGER',
            (True, False, False, False): 'OBJECT_TRIGGER',
            (False, True, False, False): 'SIGN_TRIGGER',
            (False, False, True, False): 'INTERSECTION_TRIGGER',
            (False, False, False, True): 'TRAFFIC_LIGHT_TRIGGER',
            (False, False, False, False): 'NO_TRIGGER'
        }

        self.state = state_map.get((object_trigger, sign_trigger, intersection_trigger, traffic_light_trigger))

        # Update instance variables for the seven triggers
        self.break_trigger =0
        self.road_search = 0
        self.switch_lane = 0
        self.parking = 0
        self.lane_follow = 0
        self.acceleration = 0
        self.intersection = 0


    def perform_action(self):
        # Perform actions based on state
        if self.state == 'OBJECT_AND_SIGN_TRIGGER':
            self.break_trigger = False
            self.road_search = False
            self.switch_lane = False
            self.parking = False
            self.lane_follow = False
            self.acceleration = False
            self.intersection = False
        # Do something when both object and sign triggers are activated
        # ...
        elif self.state == 'OBJECT_TRIGGER':
            self.break_trigger = True
            self.road_search = False
            self.switch_lane = False
            self.parking = False
            self.lane_follow = False
            self.acceleration = False
            self.intersection = False
            self.override = True
        # Do something when only object trigger is activated
        # ...
        elif self.state == 'SIGN_TRIGGER':
            self.break_trigger = False
            self.road_search = False
            self.switch_lane = False
            self.parking = False
            self.lane_follow = False
            self.acceleration = False
            self.intersection = False
        # Do something when only sign trigger is activated
        # ...
        elif self.state == 'INTERSECTION_TRIGGER':
            self.break_trigger = False
            self.road_search = False
            self.switch_lane = False
            self.parking = False
            self.lane_follow = False
            self.acceleration = False
            ## READ INSTRUCTIONS OTHERWISE LEFT
            self.intersection = "left"

        # Do something when intersection trigger is activated
        # ...
        elif self.state == 'TRAFFIC_LIGHT_TRIGGER':
            self.break_trigger = False
            self.road_search = False
            self.switch_lane = False
            self.parking = False
            self.lane_follow = False
            self.acceleration = False
            self.intersection = False
        # Do something when traffic light trigger is activated
        # ...
        elif self.state == 'NO_TRIGGER':
            self.break_trigger = False
            self.road_search = False
            self.switch_lane = False
            self.parking = False
            self.lane_follow = Pscene.deviation
            self.acceleration = False
            self.intersection = False
        # Do something when no triggers are activated
        # ...

        # Return array of seven triggers
        return [self.break_trigger, self.road_search, self.switch_lane, self.parking, self.lane_follow, self.acceleration,
                self.intersection]



    def __str__(self):
        return f"Brain state: {self.state}\n" + \
               f"Break trigger: {self.break_trigger}\n" + \
               f"Road search: {self.road_search}\n" + \
               f"Switch lane: {self.switch_lane}\n" + \
               f"Parking: {self.parking}\n" + \
               f"Lane follow: {self.lane_follow}\n" + \
               f"Acceleration: {self.acceleration}\n" + \
               f"Intersection: {self.intersection}\n"

#brainTEST = Brain.from_values(True, 30, True, False, "red", (0, 0, 0), "north")

# print the brain object to see the diagnostics
#print(brain)

