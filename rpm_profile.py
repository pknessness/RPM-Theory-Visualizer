import random

x = 0
y = 0

def executeTorque(desired_g: float):
    global x, y
    x += (random.random() - 0.5) * 0.2
    y += (random.random() - 0.5) * 0.2
    return [x, y]
    #return [10,10]
    
def executePos(desired_g: float):
    global x, y
    x += (random.random() - 0.5) * 0.2
    y += (random.random() - 0.5) * 0.2
    return [x, y]

def executeTrajectory(desired_g: float, outer_theta: float, inner_theta: float):
    global x, y
    x += (random.random() - 0.5) * 0.2
    y += (random.random() - 0.5) * 0.2
    return [x, y]

def executeVelo(desired_g: float):
    global x, y
    x += (random.random() - 0.5) * 0.2
    y += (random.random() - 0.5) * 0.2
    return [x, y]
    #return [10,10]