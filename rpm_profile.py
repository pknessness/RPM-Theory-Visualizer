import random, math
from constants import *

x = 0
y = 0

def norm2(vec: list):
    magn = mag2(vec)
    return [vec[0]/magn, vec[1]/magn]

def norm3(vec: list):
    magn = mag3(vec)
    return [vec[0]/magn, vec[1]/magn, vec[2]/magn]

def seed(seed: int):
    random.seed(seed)

def executeTorque(elapsed_time: float, desired_g: float):
    global x, y
    x += (random.random() - 0.5) * 0.2
    y += (random.random() - 0.5) * 0.2
    return [x, y]
    #return [10,10]
    
def executePos(elapsed_time: float, desired_g: float):
    global x, y
    x += (random.random() - 0.5) * 0.2
    y += (random.random() - 0.5) * 0.2
    return [x, y]

def executeTrajectory(elapsed_time: float, desired_g: float, outer_theta: float, inner_theta: float):
    global x, y
    x += (random.random() - 0.5) * 0.2
    y += (random.random() - 0.5) * 0.2
    return [x, y]

velo_lastChange = 0
velo_changeDT = 0.3
velo_angle = 0
velo_angleCone = 15
velo_maxVelocity = VELO_MAX
def executeAnshal(elapsed_time: float, desired_g: float):
    global x, y, velo_lastChange, velo_angle, velo_changeDT, velo_angleCone, velo_maxVelocity
    if(elapsed_time > (velo_lastChange + velo_changeDT)):
        velo_lastChange += velo_changeDT
        velo_angle += (random.random() * velo_angleCone) - (velo_angleCone/2)
        x = math.sin(math.radians(velo_angle)) * velo_maxVelocity
        y = math.cos(math.radians(velo_angle)) * velo_maxVelocity
    return [x, y]
    #return [10,10]
    
def executeJon(elapsed_time: float, desired_g: float):
    norm = norm2([math.pi, math.e])
    return [norm[0]*VELO_MAX, norm[1]*VELO_MAX]

def executeVelo(elapsed_time: float, desired_g: float):
    return executeAnshal(elapsed_time, desired_g)
    #return [10,10]