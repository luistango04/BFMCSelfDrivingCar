from SCENE import PScene
from Setup import BFMC_MQTT_CONTROL_TOPIC

class Brain:

    def __init__(self, PScene, jsonReader=None):

        # Initialize instance variables for the seven triggers
        self.brake_trigger = False
        self.road_search = False
        self.switch_lane = False
        self.parking = False
        self.lane_follow = False
        self.acceleration = False
        self.intersection = False
        self.override = False ## EMERGENCY FLAG
        self.deviation = PScene.deviation
        self.distancetocar = PScene.distancetocar
        self.direction = PScene.direction
        self.stop_trigger = False
        self.targetdistance = 50
        self.jsonReader = jsonReader
        self.system_start = True #this guy is immediately set to false
        self.current_mqtt_message = None

        if PScene is not None:
            self.update(PScene)


    def update(self, PScene):
        object_trigger = PScene.get_object_trigger()
        sign_trigger = PScene.get_sign_trigger()
        intersection_trigger = PScene.intersection_trigger
        traffic_light_trigger = PScene.traffic_light_trigger
        self.deviation = PScene.deviation
        self.distancetocar = PScene.distancetocar
        stopsign  = PScene.stop_trigger

        #if self.jsonReader != None:
        #    print("GOT MQTT MESSAGE")
        #    tmp = self.jsonReader.get_next_message(BFMC_MQTT_CONTROL_TOPIC, self.system_start)
        #    if tmp != None:
        #        self.current_mqtt_message = tmp
        #    self.system_start = False

        #stopsign= True ## Eeddid otu when this works
        if(abs(self.deviation) > 20):
            lancorrect = True
        else:
            lancorrect = False
        if(self.distancetocar ):
            print("Car far away")
            cruisecontrol = True
        else:
            print("No Car detected")
            cruisecontrol = True


        state_map = {
            (True, True, False, False,False,False,False): 'OBJECT_AND_SIGN_TRIGGER',
            (True, True, False, False, False,False,False): 'OBJECT_TRIGGER',
            (True, True, False, False, False,False,False): 'SIGN_TRIGGER',
            (False, False, True, False, False,False,False): 'INTERSECTION_TRIGGER',
            (False, False, True, False, False,True,True): 'INTERSECTION AND STOP SIGN',
            (True, True, False, False, False,False,False): 'TRAFFIC_LIGHT_TRIGGER',
            (True, True, False, False, False,False,False): 'NO_TRIGGER',
            (False, False, False, False, True,False,False): 'LANE_CORRECTION',
            (False, False, False, False, False, False, True): 'CRUISECONTROL',
            (False, False, True, False, True, False, True): 'CRUISECONTROL',
            (False, False, False, False, True, False, True): 'CRUISECONTROL',

            (True, True, False, False, False,False,False): 'OBJECT_AND_INTERSECTION_TRIGGER',
        }

        #if self.current_mqtt_message is not None and self.current_mqtt_message in state_map.values():
        #    self.state = self.current_mqtt_message
        #    print("Executing state: " + self.current_mqtt_message)
    #    else:
            #print(state_map.get((object_trigger, sign_trigger, intersection_trigger, traffic_light_trigger,lancorrect)))
        self.state = state_map.get((object_trigger, sign_trigger, intersection_trigger, traffic_light_trigger,lancorrect,stopsign,cruisecontrol))

        # Update instance variables for the seven triggers
        self.brake_trigger = 0
        self.road_search = 0
        self.switch_lane = 0
        self.parking = 0
        self.lane_follow = 0
        self.acceleration = 0
        self.intersection = 0


    def perform_action(self):
        # Perform actions based on state

        # Do something when no triggers are activated
        # ...
        # Return array of seven triggers
        return [self.brake_trigger, self.road_search, self.switch_lane, self.parking, self.lane_follow, self.acceleration,
                self.intersection]

#    def fetchactivities(self,connectionobject): ## Put planned activities here in brain

    def __str__(self):
        return f"Brain state: {self.state}\n" + \
               f"brake trigger: {self.brake_trigger}\n" + \
               f"Road search: {self.road_search}\n" + \
               f"Switch lane: {self.switch_lane}\n" + \
               f"Parking: {self.parking}\n" + \
               f"Lane follow: {self.lane_follow}\n" + \
               f"Acceleration: {self.acceleration}\n" + \
               f"Deviation: {self.deviation}\n" + \
               f"Direction: {self.direction}\n" + \
               f"Distancetocar: {self.distancetocar}\n" + \
               f"Target: {self.targetdistance}\n" + \
               f"Intersection: {self.intersection}\n"


#brainTEST = Brain.from_values(True, 30, True, False, "red", (0, 0, 0), "north")

# print the brain object to see the diagnostics
#print(brain)

