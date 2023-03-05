from SCENE import PScene
class Brain:
    def __init__(self, PScene):
        if(PScene is not None):

            self.Objecttrigger = PScene.get_object_trigger()
            self.midlane = PScene.deviation
            self.signtrigger =  PScene.get_sign_trigger()
            self.intersectiontrigger =  PScene.intersection_trigger
            self.trafficlight = PScene.traffic_light_trigger
            self.posandrotation =  PScene.get_position()
            self.intersection_navigation_trigger = PScene.position
        self.breaktrigger = 0
        self.roadsearch = 0
        self.switchlane = 0
        self.parkingtrigger = 0
        self.errortomid = PScene.deviation
        self.speed =  0
        self.turn = 0

    def update_from_scene(self, PScene):
        if PScene is not None:
            self.Objecttrigger = PScene.get_object_trigger()
            self.midlane = PScene.deviation
            self.signtrigger = PScene.get_sign_trigger()
            self.intersectiontrigger = PScene.intersection_trigger
            self.trafficlight = PScene.traffic_light_trigger
            self.posandrotation = PScene.get_position()
            self.intersection_navigation_trigger = PScene.position
            self.errortomid = PScene.deviation

    @classmethod
    def from_values(cls, Objecttrigger, midlane, signtrigger, intersectiontrigger, trafficlight, posandrotation, intersection_navigation_trigger):
        brain = cls(None)
        brain.Objecttrigger = Objecttrigger
        brain.midlane = midlane
        brain.signtrigger = signtrigger
        brain.intersectiontrigger = intersectiontrigger
        brain.trafficlight = trafficlight
        brain.posandrotation = posandrotation
        brain.intersection_navigation_trigger = intersection_navigation_trigger
        return brain

    def __str__(self):
      info = f"Objecttrigger: {self.Objecttrigger}\n"
      info += f"midlane: {self.midlane}\n"
      info += f"signtrigger: {self.signtrigger}\n"
      info += f"intersectiontrigger: {self.intersectiontrigger}\n"
      info += f"trafficlight: {self.trafficlight}\n"
      info += f"posandrotation: {self.posandrotation}\n"
      info += f"intersection_navigation_trigger: {self.intersection_navigation_trigger}\n"
      info += f"breaktrigger: {self.breaktrigger}\n"
      info += f"roadsearch: {self.roadsearch}\n"
      info += f"breaktrigger: {self.breaktrigger}\n"
      info += f"errortomid: {self.errortomid}\n"
      info += f"intersection_navigation_trigger: {self.intersection_navigation_trigger}\n"
      return info

class FSM:
    def __init__(self, states, start_state):
        self.states = states
        self.state = start_state

    def run(self, input):
        (new_state, output) = self.states[self.state](input)
        self.state = new_state
        return output


# create a Brain instance using the from_values method
#brainTEST = Brain.from_values(True, 30, True, False, "red", (0, 0, 0), "north")

# print the brain object to see the diagnostics
#print(brain)

