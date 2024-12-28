#CONSTANTS
sample_frequency = 10
dt = 1/sample_frequency
speed_mult = 1

#kg m^2
OUTER_MOI = 15.0
INNER_MOI = 10.0

OUTER_FRICTION_FORCE = 1
INNER_FRICTION_FORCE = 1

#mm
OUTER_SIDE_LENGTH = 50.0 
INNER_SIDE_LENGTH = 40.0
PLATFORM_SIDE_LENGTH = 30.0

FRAME_THICKNESS = 2.0  

class WriteLine:
    def __init__(self, text, color):
        self.text = text
        self.color = color
            
    def __str__(self):
        return f"{self.text}"