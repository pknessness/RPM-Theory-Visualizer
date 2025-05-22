import math, time
from constants import *
import render
import rpm_profile as rpm_profile
import pygame
from datetime import datetime, timezone
import signal
import sys

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

elapsed_time = 0

beginTime = 0

def secToStr(seconds: float):
    ms = f"{seconds:.3f}".split('.')[1]
    sec = int(seconds) % 60
    min = int(seconds / 60) % 60
    hr = int(seconds / 60 / 60)
    return f"{hr}:{min}:{sec}.{ms}"

def endPrint():
    print(f"\nSimulated Time: {secToStr(elapsed_time)} Effective Accel: {mag3(accel)}G Realtime: {secToStr(time.time() - beginTime)}")
    

def signal_handler(signal, frame):
    endPrint()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

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

POS_PID_ONLY_ENABLED = False #NOT RECOMMENDED TO USE
POS_VELO_PID_ENABLED = False
VELO_PID_ONLY_ENABLED = True

MANUAL_CONTROL = False
MANUAL_POS_SCALING = 0.01
MANUAL_TORQUE_SCALING = 1

MANUAL_VELO_SCALING = 0.01

doRender = True

PROFILE_VELO_ANSHAL = False # SIMPLIFIED RANDOM DIRECTION ALGORITHM
PROFILE_VELO_ANSHALMODIFIED = False # SIMPLIFIED RANDOM DIRECTION ALGORITHM (WITH ACOS)
PROFILE_VELO_JON = False # INCOMMENSURABLE NUMBERS ALGORITHM
PROFILE_VELO_JONMODIFIED = False # CONSISTENTLY CHANGING DIRECTION ALGORITHM
PROFILE_VELO_BRW = True # BOUNDED RANDOM WALK ALGORITHM

StopAtXSeconds = 60 * 60 * 1 # 1hr

filename = "blank"


writeBuffer = ""
writeFileCount = 0
def flushBuffer():
    global writeBuffer, writeFileCount
    f = open(filename + ".csv", "a")
    f.write(writeBuffer)
    f.close()
    writeBuffer = ""
    writeFileCount = 0

def writeFile(text):
    global writeBuffer, writeFileCount
    writeBuffer += text
    writeFileCount += 1
    if(writeFileCount > 1000):
        flushBuffer()

beginTime = 0

if __name__ == "__main__":
    rand_seed(0)
    
    if(doRender or MANUAL_CONTROL):
        render.init()
    
    print(f"STARTING SIMULATION\nSAMPLING_HZ: {sample_frequency}Hz SAMPLING_PERIOD: {dt}s SPEED_MULT: {speed_mult}\n MOI (kgm^2): OUTER: {OUTER_MOI} INNER: {INNER_MOI}\n")
    
    prev_time = time.time()
    beginTime = time.time()
    
    outer_pid_pos = PID(10,0,4)
    inner_pid_pos = PID(10,0,4)
    
    cascade_outer_pid_velo = PID(50,0,5)
    cascade_outer_pid_velo = PID(50,0,5)
    cascade_outer_pid_pos = PID(50,0,0)
    cascade_outer_pid_pos = PID(50,0,0)
    
    desired_outer_theta, desired_inner_theta, outer_torque, inner_torque = [0,0,0,0]
    
    prev_outer_theta = 0
    prev_inner_theta = 0
    
    prevMouse = [False, False, False]
    dragPos = [[0,0],[0,0],[0,0]]
    
    now = datetime.now()
    filename = "logs/log_"+now.strftime("%d-%m-%Y_%H_%M_%S")
    if(POS_PID_ONLY_ENABLED):
        filename += f"_P"
    elif(POS_VELO_PID_ENABLED):
        filename += f"_PV"
    elif(VELO_PID_ONLY_ENABLED):
        filename += f"_V"
        if(PROFILE_VELO_ANSHAL):
            filename += f"_ANSH_{rpm_profile.veloAnshal_changeDT*100}ms_{rpm_profile.veloAnshal_angleCone}deg_{rpm_profile.veloAnshal_maxVelocity}rads"
        elif(PROFILE_VELO_ANSHALMODIFIED):
            filename += f"_ANSHMODIFIED_{rpm_profile.veloAnshal_changeDT*100}ms_{rpm_profile.veloAnshal_angleCone}deg_{rpm_profile.veloAnshal_maxVelocity}rads"
        elif(PROFILE_VELO_JON):
            filename += f"_JON_{rpm_profile.veloJon2_maxVelocity}rads"
        elif(PROFILE_VELO_JONMODIFIED):
            filename += f"_JONMODIFIED_{rpm_profile.veloJon2_timePerRotation}s_{rpm_profile.veloJon2_maxVelocity}rads"
        elif(PROFILE_VELO_BRW):
            filename += f"_BRW_{rpm_profile.veloJon2_timePerRotation}_{rpm_profile.veloBRW_coneAngle}rad_{rpm_profile.veloBRW_coneLength}len_{rpm_profile.veloBRW_maxVelocity}rads"
        else:
            print("NO PROFILE SELECTED, SET ONE OF THE PROFILE_VELO_ VARIABLES TO TRUE")
            exit(-5)
    else:
        print("NO MODE SELECTED, SET ONE OF THE THREE _ENABLED VARIABLES TO TRUE")
        exit(-3)
    
    #filename += f"-{VELO_MAX}RADs-"
    
    if(MANUAL_CONTROL):
        filename += "_MAN"
    
    writeFile("time (ms), inner (rad), outer (rad), accel_x, accel_y, accel_z, accel_effective\n")
    
    while(1):
        if(doRender or MANUAL_CONTROL):
            mouse = pygame.mouse.get_pressed(num_buttons=3)
        
        prev_outer_omega = (outer_theta - prev_outer_theta)/dt
        prev_inner_omega = (inner_theta - prev_inner_theta)/dt
        
        prev_outer_theta = outer_theta
        prev_inner_theta = inner_theta
        
        renderPts = []
        
        if(POS_PID_ONLY_ENABLED):
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
        elif(POS_VELO_PID_ENABLED):
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
                #desired_outer_theta, desired_inner_theta = rpm_profile.executePos(elapsed_time, 0.0)
                
                #it goes azimuth, elevation, and inner is azimuth and outer is elevation so inner, outer on the input
                desired_outer_theta, desired_inner_theta, renderPts = rpm_profile.executeBoundedRandomVelocity(elapsed_time, 0.0, [prev_inner_theta, prev_outer_theta])
                #print(renderPts)
            desired_outer_omega = cascade_outer_pid_pos.calculate(delta(outer_theta, desired_outer_theta, 2*math.pi), 0, dt)
            desired_inner_omega = cascade_outer_pid_pos.calculate(delta(inner_theta, desired_inner_theta, 2*math.pi), 0, dt)
            
            mag = mag2([desired_outer_omega, desired_inner_omega])

            if(mag != 0):
                desired_outer_omega = desired_outer_omega * VELO_MAX/mag
                desired_inner_omega = desired_inner_omega * VELO_MAX/mag
            outer_torque = cascade_outer_pid_velo.calculate(desired_outer_omega, prev_outer_omega, dt)
            inner_torque = cascade_outer_pid_velo.calculate(desired_inner_omega, prev_inner_omega, dt)
            
            executePhysics(outer_torque, inner_torque, dt)
        elif(VELO_PID_ONLY_ENABLED):
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
                if(PROFILE_VELO_ANSHAL):
                    desired_outer_omega, desired_inner_omega = rpm_profile.executeAnshal(elapsed_time, 0.0)
                elif(PROFILE_VELO_ANSHALMODIFIED):
                    desired_outer_omega, desired_inner_omega = rpm_profile.executeAnshalModified(elapsed_time, 0.0)
                elif(PROFILE_VELO_JON):
                    desired_outer_omega, desired_inner_omega = rpm_profile.executeJon(elapsed_time, 0.0)
                elif(PROFILE_VELO_JONMODIFIED):
                    desired_outer_omega, desired_inner_omega = rpm_profile.executeJonModified(elapsed_time, 0.0)
                elif(PROFILE_VELO_BRW):
                    desired_outer_omega, desired_inner_omega, renderPts = rpm_profile.executeBoundedRandomVelocity(elapsed_time, 0.0, [prev_inner_theta, prev_outer_theta])
                else:
                    print("SHOULD NEVER REACH")
                    exit(-6)
                    
            desired_outer_omega = max(-VELO_MAX, min(VELO_MAX, desired_outer_omega))
            desired_inner_omega = max(-VELO_MAX, min(VELO_MAX, desired_inner_omega))
            outer_torque = cascade_outer_pid_velo.calculate(desired_outer_omega, prev_outer_omega, dt)
            inner_torque = cascade_outer_pid_velo.calculate(desired_inner_omega, prev_inner_omega, dt)
                
            executePhysics(outer_torque, inner_torque, dt)
        else:
            print("SHOULD NEVER REACH")
            exit(-4)

        executeGCalcs()
        
        projectionEffectiveG()
        
        #print(f"[POST_CYCLE_SUMMARY {elapsed_time:.2f}s] OUTER: {outer_theta:.2f}/{fmod(outer_theta,2*math.pi):.2f} INNER: {inner_theta:.2f}/{fmod(inner_theta,2*math.pi):.2f} OUTER': {outer_dtheta:.2f} INNER': {inner_dtheta:.2f}")
        if(doRender or MANUAL_CONTROL):
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
            if(POS_PID_ONLY_ENABLED):
                desired = [desired_outer_theta,desired_inner_theta]
                renderings.append(WriteLine(f"OUTER PID: P:{outer_pid_pos.pComp:.2f} I:{outer_pid_pos.iComp:.2f} D:{outer_pid_pos.dComp:.2f}", (155,150,200)))
                renderings.append(WriteLine(f"INNER PID: P:{inner_pid_pos.pComp:.2f} I:{inner_pid_pos.iComp:.2f} D:{inner_pid_pos.dComp:.2f}", (155,150,200)))
            elif(POS_VELO_PID_ENABLED):
                desired = [desired_outer_theta,desired_inner_theta]
                renderings.append(WriteLine(f"OUTER PID V: {desired_outer_omega:.2f}/{prev_outer_omega:.2f} P:{cascade_outer_pid_velo.pComp:.2f} I:{cascade_outer_pid_velo.iComp:.2f} D:{cascade_outer_pid_velo.dComp:.2f}", (155,150,200)))
                renderings.append(WriteLine(f"INNER PID V: {desired_inner_omega:.2f}/{prev_inner_omega:.2f} P:{cascade_outer_pid_velo.pComp:.2f} I:{cascade_outer_pid_velo.iComp:.2f} D:{cascade_outer_pid_velo.dComp:.2f}", (155,150,200)))
                renderings.append(WriteLine(f"OUTER PID P: P:{cascade_outer_pid_pos.pComp:.2f} I:{cascade_outer_pid_pos.iComp:.2f} D:{cascade_outer_pid_pos.dComp:.2f}", (155,150,200)))
                renderings.append(WriteLine(f"INNER PID P: P:{cascade_outer_pid_pos.pComp:.2f} I:{cascade_outer_pid_pos.iComp:.2f} D:{cascade_outer_pid_pos.dComp:.2f}", (155,150,200)))
            
            renderings.append(WriteLine(f"EFFECTIVE: X:{accel[0]:.2f} Y:{accel[1]:.2f} Z:{accel[2]:.2f}", (205,100,200)))
            renderings.append(WriteLine(f"INSTANTANEOUS: X:{instaccel[0]:.2f} Y:{instaccel[1]:.2f} Z:{instaccel[2]:.2f}", (205,100,200)))
            
            
            #[accel[0] * scalar, accel[1] * scalar, accel[2] * scalar]
            # render.render([outer_theta, inner_theta], desired, 
            #                 [render.vec3D(instaccel, [0,0,0], 9.81, False)], 
            #                 renderings)
        
            render.render([outer_theta, inner_theta], desired, 
                [render.vec3D(accel, [0,0,0], 9.81, False)], 
                renderings, renderPts)
        
        writeFile(f"{elapsed_time:.2f}, {inner_theta:.6f}, {outer_theta:.6f}, {instaccel[0]:.6f}, {instaccel[1]:.6f}, {instaccel[2]:.6f}, {mag3(accel):.6f}\n")
        
        elapsed_time += dt
        if(speed_mult == 1):
            cur_time = time.time()
            #print(cur_time-prev_time)
            if((cur_time - prev_time) < dt):
                time.sleep(dt - (cur_time - prev_time))
        prev_time = time.time()
        
        if(StopAtXSeconds != 0 and elapsed_time > StopAtXSeconds):
            endPrint()
            exit()
        
        if(doRender or MANUAL_CONTROL):
            prevMouse = mouse
            
        if(StopAtXSeconds != 0 and int(elapsed_time*100) == int(elapsed_time)*100 and int(elapsed_time) % 60 == 0):
            print(f"\rProgress: {elapsed_time/StopAtXSeconds * 100 :.2f}% {secToStr((prev_time - beginTime) * (StopAtXSeconds - elapsed_time) / elapsed_time)} remaining", end="")