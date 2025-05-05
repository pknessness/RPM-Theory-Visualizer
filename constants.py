#CONSTANTS
sample_frequency = 100
dt = 1/sample_frequency
speed_mult = 1

#kg m^2
OUTER_MOI = 15.0
INNER_MOI = 10.0

#Nm
OUTER_FRICTION_FORCE = 0.5
INNER_FRICTION_FORCE = 0.5

#mm
OUTER_SIDE_LENGTH = 50.0 
INNER_SIDE_LENGTH = 40.0
PLATFORM_SIDE_LENGTH = 30.0

FRAME_THICKNESS = 2.0  


#profile (rad/s)
VELO_MAX = 0.733 

class WriteLine:
    def __init__(self, text, color):
        self.text = text
        self.color = color
            
    def __str__(self):
        return f"{self.text}"
    
def fmod(num: float, base: float):
    op = num
    while(op > base):
        op -= base
    while(op < 0):
        op += base
    return op