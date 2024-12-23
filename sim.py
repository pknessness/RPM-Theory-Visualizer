import math, time

#CONSTANTS
sample_frequency = 10
dt = 1/sample_frequency
speed_mult = 0

#kg m^2
OUTER_MOI = 15.0
INNER_MOI = 10.0

OUTER_FRICTION_FORCE = 1
INNER_FRICTION_FORCE = 1

#mm
OUTER_SIDE_LENGTH = 40.0 
INNER_SIDE_LENGTH = 30.0

FRAME_THICKNESS = 2.0  

#VARIABLES
outer_theta = 0.0
inner_theta = 0.0

outer_dtheta = 0.0
inner_dtheta = 0.0

# outer_d2theta = 0
# inner_d2theta = 0

#results
accel = [0.0,0.0,0.0]

#RETURNS [desired_outer_theta, desired_inner_theta] 
def profile(desired_g: float):
    return [0,0]

class PID:
    def __init__(self, kP: float, kI: float, kD: float):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prevError = 0
        self.sumError = 0
        
    def calculate(desired: float, actual: float, dt: float):
        return 0.0

#RETURNS [outer_d2theta, inner_d2theta] 
def PID(desired, actual, dt):
    return [0,0]

def executePhysics(outer_torque: float, inner_torque: float, delta_time: float):
    global outer_theta, inner_theta, outer_dtheta, inner_dtheta
    
    #torque -> angular velocity
    outer_dtheta += (outer_torque/OUTER_MOI) * delta_time
    inner_dtheta += (inner_torque/INNER_MOI) * delta_time
    
    #angular velocity -> angle
    outer_theta += outer_dtheta * delta_time
    inner_theta += inner_dtheta * delta_time
    
    #kinematic friction
    outer_dir = 0
    if(outer_dtheta < 0): outer_dir = -1
    elif(outer_dtheta > 0): outer_dir = 1
    inner_dir = 0
    if(inner_dtheta < 0): inner_dir = -1
    elif(inner_dtheta > 0): inner_dir = 1
    
    # outer_friction = -outer_dir * OUTER_FRICTION_FORCE
    # inner_friction = -inner_dir * INNER_FRICTION_FORCE
    
    # if(outer_dtheta > )
    return

def fmod(num: float, base: float):
    op = num
    while(op > base):
        op -= base
    while(op < 0):
        op += base
    return op

if __name__ == "__main__":
    print(f"STARTING SIMULATION\nSAMPLING_HZ: {sample_frequency} SAMPLING_PERIOD: {dt} SPEED_MULT: {speed_mult}\n MOI (kgm^2): OUTER: {OUTER_MOI} INNER: {INNER_MOI}\n")
    elapsed_time = 0
    while(1):
        desired_outer_theta, desired_inner_theta = profile(0.0)
        outer_torque, inner_torque = 0.01,0.01
        executePhysics(outer_torque, inner_torque, dt)
        print(f"[POST_CYCLE_SUMMARY {elapsed_time:.2f}s] OUTER:{outer_theta:.2f}/{fmod(outer_theta,2*math.pi):.2f} INNER:{inner_theta}/{fmod(inner_theta,2*math.pi):.2f} OUTER':{outer_dtheta:.2f} INNER':{inner_dtheta:.2f}")
        elapsed_time += dt
        if(speed_mult == 0):
            time.sleep(dt)