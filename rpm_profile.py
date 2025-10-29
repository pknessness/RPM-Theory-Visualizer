import random, math
from constants import *

x = 0
y = 0

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

trail = []

def addTrail(pos_sph: list):
    trail.append(sph2cart(*pos_sph))
    if(len(trail) > 500):
        trail.pop(0)

veloAnshal_lastChange = 0
veloAnshal_changeDT = 0.3 #sec
veloAnshal_angle = 0 #degrees
veloAnshal_angleCone = 1 #degrees
veloAnshal_maxVelocity = VELO_MAX #rad/s
def executeAnshal(elapsed_time: float, desired_g: float, pos_sph: list):
    global x, y, veloAnshal_lastChange, veloAnshal_angle, veloAnshal_changeDT, veloAnshal_angleCone, veloAnshal_maxVelocity
    if(elapsed_time > (veloAnshal_lastChange + veloAnshal_changeDT)):
        veloAnshal_lastChange += veloAnshal_changeDT
        veloAnshal_angle += (random.random() * veloAnshal_angleCone) - (veloAnshal_angleCone/2)
        x = math.sin(math.radians(veloAnshal_angle)) * veloAnshal_maxVelocity
        y = math.cos(math.radians(veloAnshal_angle)) * veloAnshal_maxVelocity
    addTrail(pos_sph)
    return [x, y, trail]
    #return [10,10]

def executeAnshalModified(elapsed_time: float, desired_g: float, pos_sph: list):
    global x, y, veloAnshal_lastChange, veloAnshal_angle, veloAnshal_changeDT, veloAnshal_angleCone, veloAnshal_maxVelocity
    if(elapsed_time > (veloAnshal_lastChange + veloAnshal_changeDT)):
        veloAnshal_lastChange += veloAnshal_changeDT
        veloAnshal_angle += (random.random() * veloAnshal_angleCone) - (veloAnshal_angleCone/2)
        x = math.sin(math.radians(veloAnshal_angle))
        y = math.cos(math.radians(veloAnshal_angle))
        x, y = norm2([2 * (math.acos(x) / math.pi) - 1, y])
        x, y = v2Scale([x, y], veloAnshal_maxVelocity)
    addTrail(pos_sph)
    return [x, y, trail]

def executeJon(elapsed_time: float, desired_g: float, pos_sph: list):
    norm = norm2([math.pi, math.e])
    addTrail(pos_sph)
    return [norm[0]*VELO_MAX, norm[1]*VELO_MAX, trail]

veloJon2_maxVelocity = VELO_MAX
veloJon2_timePerRotation = 60 * 10 #sec
def executeJonModified(elapsed_time: float, desired_g: float, pos_sph: list):
    velo_angle = 2 * math.pi * (elapsed_time % veloJon2_timePerRotation) / veloJon2_timePerRotation + math.pi
    x = math.sin(math.radians(velo_angle)) * veloJon2_maxVelocity
    y = math.cos(math.radians(velo_angle)) * veloJon2_maxVelocity
    addTrail(pos_sph)
    return [x, y, trail]

veloClinostat_maxVelocity = VELO_MAX
def executeClinostat(elapsed_time: float, desired_g: float, pos_sph: list):
    x = veloClinostat_maxVelocity
    y = 0
    addTrail(pos_sph)
    return [x, y, trail]

# def executeVelo(elapsed_time: float, desired_g: float):
#     return executeAnshal(elapsed_time, desired_g)
    #return [10,10]

veloBRW_lastChange = 0
veloBRW_changeDT = 1 #sec
veloBRW_pts = []
veloBRW_maxVelocity = VELO_MAX
veloBRW_coneAngle = 15 * math.pi / 180 # Radial angle of the cone in radians (Half of max angle of cone)
veloBRW_coneLength = 0.55
veloBRW_coneMinLength = 0.1 
for i in range(50000):
    veloBRW_pts.append(generateUniformSpherePoint())
    #veloBRW_pts.append(generateWeightedSpherePoint(1))

veloBRW_prevPos = [0,0]
veloBRW_pointsInRange = []
veloBRW_coneLengthSquared = veloBRW_coneLength*veloBRW_coneLength #squared for performance
veloBRW_coneMinLengthSquared = veloBRW_coneMinLength*veloBRW_coneMinLength #squared for performance
def executeBoundedRandomVelocity(elapsed_time: float, desired_g: float, pos_sph: list):
    global veloBRW_pts, veloBRW_maxVelocity, veloBRW_coneLengthSquared, veloBRW_coneMinLengthSquared, veloBRW_coneAngle, veloBRW_prevPos, veloBRW_lastChange, veloBRW_changeDT, x, y, veloBRW_pointsInRange
    if(elapsed_time > (veloBRW_lastChange + veloBRW_changeDT)):
        veloBRW_pointsInRange = []
        veloBRW_lastChange += veloBRW_changeDT
        prev_pos_cart = sph2cart(*veloBRW_prevPos)
        pos_cart = sph2cart(*pos_sph)
        #if(pos_sph != [0,0]):
        #    print(pos_sph, pos_cart, cart2sph(*pos_cart))
        velo = v3Scale(v3Sub(pos_cart, prev_pos_cart), 1 / dt)
        for candidatePt in veloBRW_pts:
            candVector = v3Sub(candidatePt, pos_cart)
            magS = magSquared3(candVector)
            if(magS < veloBRW_coneLengthSquared and magS > veloBRW_coneMinLengthSquared):
                if(magSquared3(velo) == 0 or v3Angle(candVector,velo) < veloBRW_coneAngle):
                    veloBRW_pointsInRange.append(candidatePt)
        if(len(veloBRW_pointsInRange) != 0):
            pt = random.choice(veloBRW_pointsInRange)
            desired = cart2sph(*pt)
            desired = fmodl(desired, 2 * math.pi)
            veloBRW_pts.remove(pt)
            veloBRW_pts.append(generateUniformSpherePoint())
            delt = [delta(pos_sph[0], desired[0], 2 * math.pi), delta(pos_sph[1], desired[1], 2 * math.pi)]
            px = x
            py = y
            x, y = norm2(delt)
            if(1):
                
                if(1 or abs(px - x) + abs(py - y) > 0.1):
                    print(f"A{pos_sph[0]:.2f},{pos_sph[1]:.2f} D{desired[0]:.2f},{desired[1]:.2f} delt{delt[0]:.2f},{delt[1]:.2f} {px:.2f},{py:.2f} -> {x:.2f},{y:.2f}")
            x, y = v2Scale([x, y], veloBRW_maxVelocity)
    veloBRW_prevPos = pos_sph
    addTrail(pos_sph)
    return [x,y, trail,veloBRW_pointsInRange]

posBRW_lastChange = 0
posBRW_changeDT = 0.02 #sec
posBRW_pts = []
posBRW_maxVelocity = VELO_MAX
posBRW_coneAngle = 15 * math.pi / 180 # Radial angle of the cone in radians (Half of max angle of cone)
posBRW_coneLength = 0.55
posBRW_coneMinLength = 0.1 
for i in range(50000):
    posBRW_pts.append(generateUniformSpherePoint())
    #veloBRW_pts.append(generateWeightedSpherePoint(1))

posBRW_prevPos = [0,0]
posBRW_pointsInRange = []
posBRW_coneLengthSquared = posBRW_coneLength*posBRW_coneLength #squared for performance
posBRW_coneMinLengthSquared = posBRW_coneMinLength*posBRW_coneMinLength #squared for performance
def executeBoundedRandomPosition(elapsed_time: float, desired_g: float, pos_sph: list):
    global posBRW_pts, posBRW_maxVelocity, posBRW_coneLengthSquared, posBRW_coneMinLengthSquared, posBRW_coneAngle, posBRW_prevPos, posBRW_lastChange, posBRW_changeDT, x, y, posBRW_pointsInRange
    if(elapsed_time > (posBRW_lastChange + posBRW_changeDT)):
        posBRW_pointsInRange = []
        posBRW_lastChange += posBRW_changeDT
        prev_pos_cart = sph2cart(*posBRW_prevPos)
        pos_cart = sph2cart(*pos_sph)
        #if(pos_sph != [0,0]):
        #    print(pos_sph, pos_cart, cart2sph(*pos_cart))
        pos = v3Scale(v3Sub(pos_cart, prev_pos_cart), 1 / dt)
        for candidatePt in posBRW_pts:
            candVector = v3Sub(candidatePt, pos_cart)
            magS = magSquared3(candVector)
            if(magS < posBRW_coneLengthSquared and magS > posBRW_coneMinLengthSquared):
                if(magSquared3(pos) == 0 or v3Angle(candVector,pos) < posBRW_coneAngle):
                    posBRW_pointsInRange.append(candidatePt)
        if(len(posBRW_pointsInRange) != 0):
            pt = random.choice(posBRW_pointsInRange)
            desired = cart2sph(*pt)
            desired = fmodl(desired, 2 * math.pi)
            posBRW_pts.remove(pt)
            posBRW_pts.append(generateUniformSpherePoint())
            #print(f"P:{} D:{} A:{}")
            x, y = desired
            #print(x, y)
            #x, y = v2Scale([x, y], posBRW_maxVelocity)
        else:
            v = v3Sub(pos_cart, prev_pos_cart)
            pos_cart += norm3(v3Scale(v, 5))
            x, y = cart2sph(pos_cart[0], pos_cart[1], pos_cart[2])
    posBRW_prevPos = pos_sph
    addTrail(pos_sph)
    return [x, y, trail, posBRW_pointsInRange]

def manualPoint(elapsed_time: float, desired_g: float, pos_sph: list):
    return [sph2cart(*pos_sph)]