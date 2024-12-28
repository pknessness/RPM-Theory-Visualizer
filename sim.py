import math, time
from constants import *
import render
import rpm_profile as rpm_profile
import pygame

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
        
        self.pComp = 0
        self.iComp = 0
        self.dComp = 0
        
    def calculate(self, desired: float, actual: float, dt: float):
        error = desired - actual
        self.pComp = self.kP * error
        self.sumError += (error * dt)
        self.iComp = self.kI * self.sumError
        self.dComp = self.kD * (error - self.prevError)/dt
        return self.pComp + self.iComp + self.dComp

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
    
    outer_friction = (outer_dir * OUTER_FRICTION_FORCE/OUTER_MOI) * delta_time
    inner_friction = (inner_dir * INNER_FRICTION_FORCE/OUTER_MOI) * delta_time
    
    if(abs(outer_dtheta) > abs(outer_friction)):
        outer_dtheta -= outer_friction
    else:
        outer_dtheta = 0
    if(abs(inner_dtheta) > abs(inner_friction)):
        inner_dtheta -= inner_friction
    else:
        inner_dtheta = 0
    
    return

def executeGCalcs():
    return 0

def fmod(num: float, base: float):
    op = num
    while(op > base):
        op -= base
    while(op < 0):
        op += base
    return op

PID_ENABLED = True

MANUAL_CONTROL = True
MANUAL_POS_SCALING = 0.01
MANUAL_TORQUE_SCALING = 1

if __name__ == "__main__":
    print(f"STARTING SIMULATION\nSAMPLING_HZ: {sample_frequency}Hz SAMPLING_PERIOD: {dt}s SPEED_MULT: {speed_mult}\n MOI (kgm^2): OUTER: {OUTER_MOI} INNER: {INNER_MOI}\n")
    elapsed_time = 0
    prev_time = time.time()
    
    outer_pid = PID(10,0,0)
    inner_pid = PID(10,0,0)
    
    desired_outer_theta, desired_inner_theta, outer_torque, inner_torque = [0,0,0,0]
    
    prevMouse = [False, False, False]
    dragPos = [[0,0],[0,0],[0,0]]
    while(1):
        mouse = pygame.mouse.get_pressed(num_buttons=3)
        
        if(PID_ENABLED):
            if(MANUAL_CONTROL):
                mouse_pos = pygame.mouse.get_pos()
                if(mouse[0] and prevMouse[0]):
                    disp = [mouse_pos[0] - dragPos[0][0], mouse_pos[1] - dragPos[0][1]]
                    desired_outer_theta += disp[1] * MANUAL_POS_SCALING
                    desired_inner_theta += disp[0] * MANUAL_POS_SCALING
                if(mouse[0]):
                    dragPos[0] = mouse_pos
                    
                # if(mouse[0] and not prevMouse[0]):
                #     dragPos
            else:
                desired_outer_theta, desired_inner_theta = rpm_profile.execute(0.0)
            
            outer_torque = outer_pid.calculate(desired_outer_theta, outer_theta, dt)
            inner_torque = inner_pid.calculate(desired_inner_theta, inner_theta, dt)
            executePhysics(outer_torque, inner_torque, dt)
        else:
            if(MANUAL_CONTROL):
                mouse_pos = pygame.mouse.get_pos()
                if(mouse[0] and prevMouse[0]):
                    disp = [mouse_pos[0] - dragPos[0][0], mouse_pos[1] - dragPos[0][1]]
                    outer_theta += disp[1] * MANUAL_POS_SCALING
                    inner_theta += disp[0] * MANUAL_POS_SCALING
                if(mouse[0]):
                    dragPos[0] = mouse_pos
                    
                # if(mouse[0] and not prevMouse[0]):
                #     dragPos
            else:
                outer_theta, inner_theta = rpm_profile.execute(0.0)
        executeGCalcs()
        print(f"[POST_CYCLE_SUMMARY {elapsed_time:.2f}s] OUTER: {outer_theta:.2f}/{fmod(outer_theta,2*math.pi):.2f} INNER: {inner_theta:.2f}/{fmod(inner_theta,2*math.pi):.2f} OUTER': {outer_dtheta:.2f} INNER': {inner_dtheta:.2f}")
        
        renderings = [
        WriteLine(f"SAMPLING_HZ: {sample_frequency} SAMPLING_PERIOD: {dt}s SPEED_MULT: {speed_mult} MOI (kgm^2): OUTER: {OUTER_MOI} INNER: {INNER_MOI}", (255,255,255)),
        WriteLine(f"ELAPSED TIME: {elapsed_time:.2f}s", (200,55,200)),
        WriteLine(f"OUTER ANGLE: {outer_theta:.2f}/{fmod(outer_theta,2*math.pi):.2f} rad", (200,55,25)),
        WriteLine(f"INNER ANGLE: {inner_theta:.2f}/{fmod(inner_theta,2*math.pi):.2f} rad", (55,200,25)),
        WriteLine(f"OUTER ANGULAR VELOCITY: {outer_dtheta:.2f} rad/s", (200,55,25)),
        WriteLine(f"INNER ANGULAR VELOCITY: {inner_dtheta:.2f} rad/s", (55,200,25)),
        WriteLine(f"OUTER TORQUE: {outer_torque:.2f} Nm", (200,55,25)),
        WriteLine(f"INNER TORQUE: {inner_torque:.2f} Nm", (55,200,25))]
        
        desired = 0
        if(PID_ENABLED):
            desired = [desired_outer_theta,desired_inner_theta]
            renderings.append(WriteLine(f"OUTER PID: P:{outer_pid.pComp:.2f} I:{outer_pid.iComp:.2f} D:{outer_pid.dComp:.2f}", (155,150,200)))
            renderings.append(WriteLine(f"INNER PID: P:{inner_pid.pComp:.2f} I:{inner_pid.iComp:.2f} D:{inner_pid.dComp:.2f}", (155,150,200)))
        
        render.render([outer_theta, inner_theta],desired, [], renderings)
        
        elapsed_time += dt
        if(speed_mult == 1):
            cur_time = time.time()
            print(cur_time-prev_time)
            if((cur_time - prev_time) < dt):
                time.sleep(dt - (cur_time - prev_time))
            prev_time = time.time()
        
        prevMouse = mouse