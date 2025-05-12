import math, time
from constants import *
import render
import rpm_profile as rpm_profile
import pygame

#VARIABLES
prev_outer_theta = 0.0
prev_inner_theta = 0.0
outer_theta = 0.0
inner_theta = 0.0

outer_dtheta = 0.0
inner_dtheta = 0.0

# outer_d2theta = 0
# inner_d2theta = 0

#results
instaccel = [0.0,-1.0,0.0]

accumulate_accel = [0.0,0.0,0.0]
accumulate_count = 0
accel = [0.0,-1.0,0.0]
scalar = 9.81

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
    global outer_theta, inner_theta, outer_theta_prev, inner_theta_prev, outer_dtheta, inner_dtheta
    
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
    global instaccel
    instaccel = [0,-1.0, 0]
        
    instaccel = render.rotateX(instaccel, outer_theta)
    instaccel = render.rotateY(instaccel, -inner_theta)
    return

def projectionEffectiveG():
    global accumulate_count, accumulate_accel, accel, instaccel
    accumulate_accel[0] += instaccel[0]
    accumulate_accel[1] += instaccel[1]
    accumulate_accel[2] += instaccel[2]
    
    accumulate_count += 1
    
    accel = [accumulate_accel[0]/accumulate_count, accumulate_accel[1]/accumulate_count, accumulate_accel[2]/accumulate_count]
    return

POS_PID_ENABLED = False
VELO_PID_ENABLED = False
VELO_ONLY_ENABLED = True

MANUAL_CONTROL = False
MANUAL_POS_SCALING = 0.01
MANUAL_TORQUE_SCALING = 1

MANUAL_VELO_SCALING = 0.01

if __name__ == "__main__":
    print(f"STARTING SIMULATION\nSAMPLING_HZ: {sample_frequency}Hz SAMPLING_PERIOD: {dt}s SPEED_MULT: {speed_mult}\n MOI (kgm^2): OUTER: {OUTER_MOI} INNER: {INNER_MOI}\n")
    elapsed_time = 0
    prev_time = time.time()
    
    outer_pid_pos = PID(10,0,4)
    inner_pid_pos = PID(10,0,4)
    
    velo_outer_pid = PID(50,0,5)
    velo_inner_pid = PID(50,0,5)
    velo_outer_pid_pos = PID(50,0,0)
    velo_inner_pid_pos = PID(50,0,0)
    
    desired_outer_theta, desired_inner_theta, outer_torque, inner_torque = [0,0,0,0]
    
    prev_outer_theta = 0
    prev_inner_theta = 0
    
    prevMouse = [False, False, False]
    dragPos = [[0,0],[0,0],[0,0]]
    while(1):
        mouse = pygame.mouse.get_pressed(num_buttons=3)
        
        prev_outer_omega = (outer_theta - prev_outer_theta)/dt
        prev_inner_omega = (inner_theta - prev_inner_theta)/dt
        
        prev_outer_theta = outer_theta
        prev_inner_theta = inner_theta
        
        if(POS_PID_ENABLED):
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
                desired_outer_theta, desired_inner_theta = rpm_profile.executePos(elapsed_time, 0.0)
            
            outer_torque = outer_pid_pos.calculate(desired_outer_theta, outer_theta, dt)
            inner_torque = inner_pid_pos.calculate(desired_inner_theta, inner_theta, dt)
            executePhysics(outer_torque, inner_torque, dt)
        elif(VELO_PID_ENABLED):
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
                desired_outer_theta, desired_inner_theta = rpm_profile.executePos(elapsed_time, 0.0)
                
            desired_outer_omega = velo_outer_pid_pos.calculate(desired_outer_theta, outer_theta, dt)
            desired_inner_omega = velo_inner_pid_pos.calculate(desired_inner_theta, inner_theta, dt)
            desired_outer_omega = max(-VELO_MAX, min(VELO_MAX, desired_outer_omega))
            desired_inner_omega = max(-VELO_MAX, min(VELO_MAX, desired_inner_omega))
            outer_torque = velo_outer_pid.calculate(desired_outer_omega, prev_outer_omega, dt)
            inner_torque = velo_outer_pid.calculate(desired_inner_omega, prev_inner_omega, dt)
            
            executePhysics(outer_torque, inner_torque, dt)
        elif(VELO_ONLY_ENABLED):
            desired_outer_omega = 0
            desired_inner_omega = 0
            if(MANUAL_CONTROL):
                mouse_pos = pygame.mouse.get_pos()
                if(mouse[0]):
                    disp = [mouse_pos[0] - dragPos[0][0], mouse_pos[1] - dragPos[0][1]]
                    desired_outer_omega = disp[1] * MANUAL_VELO_SCALING
                    desired_inner_omega = disp[0] * MANUAL_VELO_SCALING
                else:
                    dragPos[0] = mouse_pos
            else:
                desired_outer_omega, desired_inner_omega = rpm_profile.executeVelo(elapsed_time, 0.0)
                    
            desired_outer_omega = max(-VELO_MAX, min(VELO_MAX, desired_outer_omega))
            desired_inner_omega = max(-VELO_MAX, min(VELO_MAX, desired_inner_omega))
            outer_torque = velo_outer_pid.calculate(desired_outer_omega, prev_outer_omega, dt)
            inner_torque = velo_outer_pid.calculate(desired_inner_omega, prev_inner_omega, dt)
                
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
                outer_theta, inner_theta = rpm_profile.execute(elapsed_time, 0.0)

        executeGCalcs()
        
        projectionEffectiveG()
        
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
        if(POS_PID_ENABLED):
            desired = [desired_outer_theta,desired_inner_theta]
            renderings.append(WriteLine(f"OUTER PID: P:{outer_pid_pos.pComp:.2f} I:{outer_pid_pos.iComp:.2f} D:{outer_pid_pos.dComp:.2f}", (155,150,200)))
            renderings.append(WriteLine(f"INNER PID: P:{inner_pid_pos.pComp:.2f} I:{inner_pid_pos.iComp:.2f} D:{inner_pid_pos.dComp:.2f}", (155,150,200)))
        elif(VELO_PID_ENABLED):
            desired = [desired_outer_theta,desired_inner_theta]
            renderings.append(WriteLine(f"OUTER PID V: {desired_outer_omega:.2f}/{prev_outer_omega:.2f} P:{velo_outer_pid.pComp:.2f} I:{velo_outer_pid.iComp:.2f} D:{velo_outer_pid.dComp:.2f}", (155,150,200)))
            renderings.append(WriteLine(f"INNER PID V: {desired_inner_omega:.2f}/{prev_inner_omega:.2f} P:{velo_inner_pid.pComp:.2f} I:{velo_inner_pid.iComp:.2f} D:{velo_inner_pid.dComp:.2f}", (155,150,200)))
            renderings.append(WriteLine(f"OUTER PID P: P:{velo_outer_pid_pos.pComp:.2f} I:{velo_outer_pid_pos.iComp:.2f} D:{velo_outer_pid_pos.dComp:.2f}", (155,150,200)))
            renderings.append(WriteLine(f"INNER PID P: P:{velo_inner_pid_pos.pComp:.2f} I:{velo_inner_pid_pos.iComp:.2f} D:{velo_inner_pid_pos.dComp:.2f}", (155,150,200)))
        
        renderings.append(WriteLine(f"EFFECTIVE: X:{accel[0]:.2f} Y:{accel[1]:.2f} Z:{accel[2]:.2f}", (205,100,200)))
        renderings.append(WriteLine(f"INSTANTANEOUS: X:{instaccel[0]:.2f} Y:{instaccel[1]:.2f} Z:{instaccel[2]:.2f}", (205,100,200)))
        
        
        #[accel[0] * scalar, accel[1] * scalar, accel[2] * scalar]
        render.render([outer_theta, inner_theta], desired, 
                        [render.vec3D([instaccel[0]*scalar,0,0]), render.vec3D([0,instaccel[1]*scalar,0]), render.vec3D([0,0, instaccel[2]*scalar])], 
                        renderings)
        
        elapsed_time += dt
        if(speed_mult == 1):
            cur_time = time.time()
            print(cur_time-prev_time)
            if((cur_time - prev_time) < dt):
                time.sleep(dt - (cur_time - prev_time))
            prev_time = time.time()
        
        prevMouse = mouse