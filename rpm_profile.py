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

veloAnshal_lastChange = 0
veloAnshal_changeDT = 0.3 #sec
veloAnshal_angle = 0 #degrees
veloAnshal_angleCone = 1 #degrees
veloAnshal_maxVelocity = VELO_MAX #rad/s
def executeAnshal(elapsed_time: float, desired_g: float):
    global x, y, veloAnshal_lastChange, veloAnshal_angle, veloAnshal_changeDT, veloAnshal_angleCone, veloAnshal_maxVelocity
    if(elapsed_time > (veloAnshal_lastChange + veloAnshal_changeDT)):
        veloAnshal_lastChange += veloAnshal_changeDT
        veloAnshal_angle += (random.random() * veloAnshal_angleCone) - (veloAnshal_angleCone/2)
        x = math.sin(math.radians(veloAnshal_angle)) * veloAnshal_maxVelocity
        y = math.cos(math.radians(veloAnshal_angle)) * veloAnshal_maxVelocity
    return [x, y]
    #return [10,10]

def executeAnshalModified(elapsed_time: float, desired_g: float):
    global x, y, veloAnshal_lastChange, veloAnshal_angle, veloAnshal_changeDT, veloAnshal_angleCone, veloAnshal_maxVelocity
    if(elapsed_time > (veloAnshal_lastChange + veloAnshal_changeDT)):
        veloAnshal_lastChange += veloAnshal_changeDT
        veloAnshal_angle += (random.random() * veloAnshal_angleCone) - (veloAnshal_angleCone/2)
        x = math.sin(math.radians(veloAnshal_angle))
        y = math.cos(math.radians(veloAnshal_angle))
        x, y = norm2([2 * (math.acos(x) / math.pi) - 1, y])
        x, y = v2Scale([x, y], veloAnshal_maxVelocity)
    return [x, y]

def executeJon(elapsed_time: float, desired_g: float):
    norm = norm2([math.pi, math.e])
    return [norm[0]*VELO_MAX, norm[1]*VELO_MAX]

veloJon2_maxVelocity = VELO_MAX
veloJon2_timePerRotation = 60 * 10 #sec
def executeJonModified(elapsed_time: float, desired_g: float):
    velo_angle = 2 * math.pi * (elapsed_time % veloJon2_timePerRotation) / veloJon2_timePerRotation + math.pi
    x = math.sin(math.radians(velo_angle)) * veloJon2_maxVelocity
    y = math.cos(math.radians(velo_angle)) * veloJon2_maxVelocity
    return [x, y]

veloClinostat_maxVelocity = VELO_MAX
def executeClinostat(elapsed_time: float, desired_g: float):
    x = veloClinostat_maxVelocity
    y = 0
    return [x, y]

def executeVelo(elapsed_time: float, desired_g: float):
    return executeAnshal(elapsed_time, desired_g)
    return [10,10]

veloBRW_lastChange = 0
veloBRW_changeDT = 1 #sec
veloBRW_pts = []
veloBRW_maxVelocity = VELO_MAX
veloBRW_coneAngle = 30 * math.pi / 180 # Radial angle of the cone in radians (Half of max angle of cone)
veloBRW_coneLength = 0.55
veloBRW_coneMinLength = 0.01 
for i in range(100000):
    veloBRW_pts.append(generateUniformSpherePoint())
    #veloBRW_pts.append(generateWeightedSpherePoint(1))
# Print Arduino PROGMEM array
# print("const float veloBRW_pts[NUM_POINTS][3] PROGMEM = {")
# for pt in veloBRW_pts:
#     print(f"  {{ {pt[0]:.6f}, {pt[1]:.6f}, {pt[2]:.6f} }},")
# print("};")
veloBRW_prevPos = [0,0]
veloBRW_pointsInRange = []
veloBRW_coneLengthSquared = veloBRW_coneLength*veloBRW_coneLength #squared for performance
veloBRW_coneMinLengthSquared = veloBRW_coneMinLength*veloBRW_coneMinLength #squared for performance
def executeBoundedRandomVelocity(elapsed_time: float, desired_g: float, pos_sph: list):
    global veloBRW_pts, veloBRW_maxVelocity, veloBRW_coneLengthSquared, veloBRW_coneMinLengthSquared, veloBRW_coneAngle, veloBRW_prevPos, veloBRW_lastChange, veloBRW_changeDT, x, y, veloBRW_pointsInRange
    
    # Wrap angles to [0, 2π] range to prevent unbounded growth
    # pos_sph = [fmod(pos_sph[0], 2*math.pi), fmod(pos_sph[1], 2*math.pi)]
    
    if(elapsed_time > (veloBRW_lastChange + veloBRW_changeDT)):
        # print("\n--- NEW VELOBRW CALCULATION ---")
        veloBRW_pointsInRange = []
        veloBRW_lastChange += veloBRW_changeDT
        
        # print(f"  1. PREV SPH: az={veloBRW_prevPos[0]:.3f}, el={veloBRW_prevPos[1]:.3f}")
        # print(f"  1. CURR SPH: az={pos_sph[0]:.3f}, el={pos_sph[1]:.3f}")
            
        prev_pos_cart = sph2cart(*veloBRW_prevPos)
        pos_cart = sph2cart(*pos_sph)
        
        # print(f"  2. PREV CART: x={prev_pos_cart[0]:.3f}, y={prev_pos_cart[1]:.3f}, z={prev_pos_cart[2]:.3f}")
        # print(f"  2. CURR CART: x={pos_cart[0]:.3f}, y={pos_cart[1]:.3f}, z={pos_cart[2]:.3f}")
        
        velo = v3Scale(v3Sub(pos_cart, prev_pos_cart), 1 / veloBRW_changeDT)
        
        # print(f"  3. CONE DIR (unscaled): x={velo[0]:.3f}, y={velo[1]:.3f}, z={velo[2]:.3f}")
        
        for candidatePt in veloBRW_pts:
            candVector = v3Sub(candidatePt, pos_cart)
            magS = magSquared3(candVector)
            if(magS < veloBRW_coneLengthSquared and magS > veloBRW_coneMinLengthSquared):
                if(magSquared3(velo) == 0 or v3Angle(candVector,velo) < veloBRW_coneAngle):
                    veloBRW_pointsInRange.append(candidatePt)
        
        # print(f"  4. Found {len(veloBRW_pointsInRange)} points in cone.")
        
        if(len(veloBRW_pointsInRange) != 0):
            pt = random.choice(veloBRW_pointsInRange)
            # print(f"  5. CHOSEN CART: x={pt[0]:.3f}, y={pt[1]:.3f}, z={pt[2]:.3f}")
            
            desired = cart2sph(*pt)
            # veloBRW_pts.remove(pt)
            # veloBRW_pts.append(generateUniformSpherePoint())
            
            # print(f"  6. DESIRED SPH: az={desired[0]:.3f}, el={desired[1]:.3f}")

            k1 = pos_cart[0]
            k2 = pos_cart[1]
            k3 = pos_cart[2]

            # k1_dot = (pt[0] - k1) / veloBRW_changeDT
            # k2_dot = (pt[1] - k2) / veloBRW_changeDT
            # k3_dot = (pt[2] - k3) / veloBRW_changeDT

            Kp = 1.0
            k1_dot = (pt[0] - k1) * Kp
            k2_dot = (pt[1] - k2) * Kp
            k3_dot = (pt[2] - k3) * Kp

            # Add this epsilon (a tiny number) to your denominators
            epsilon = 1e-6 
            denominator_sq = (k1*k1 + k2*k2)
            denominator_sqrt = math.sqrt(denominator_sq)
            
            # Check for singularity (when near poles)
            # if denominator_sq < 0.01:  # When k1^2 + k2^2 is very small (near poles)
            #     print(f"WARNING: Approaching singularity! denominator_sq = {denominator_sq:.6f}")

            # #outer
            q1_dot = ((k1*k2_dot) - (k2*k1_dot)) / (denominator_sq + epsilon)
            # #inner
            q2_dot = -k3_dot / (math.sqrt(denominator_sq) + epsilon)


            # if (denominator_sq < 1e-5):
            #     q1_dot = 0; # Rotation around z-axis is undefined, so set to 0
            #     q2_dot = -k3_dot / (denominator_sqrt + epsilon); # This might still be large, but it's the only valid motion
            # else:
            #     q1_dot = ((k1*k2_dot) - (k2*k1_dot)) / (denominator_sq + epsilon)
            #     q2_dot = -k3_dot / (denominator_sqrt + epsilon)

            # x outer y inner
            x, y = norm2([q2_dot,q1_dot])
            
            # print(f"  8. NORM VEC: az={x:.3f}, el={y:.3f}")
            
            x, y = v2Scale([x, y], veloBRW_maxVelocity)
            # print(f"  9. FINAL VELO: x={x:.3f}, y={y:.3f}")
        else:
            print("  WARN: no points found")
        veloBRW_prevPos = pos_sph
    return [x,y,veloBRW_pointsInRange]

# veloBRW_lastChange = 0
# veloBRW_changeDT = 1 #sec - how often to pick a new target point
# veloBRW_pts = []
# veloBRW_maxVelocity = VELO_MAX
# veloBRW_coneAngle = 5 * math.pi / 180 # Radial angle of the cone in radians (Half of max angle of cone)
# veloBRW_coneLength = 0.55
# veloBRW_coneMinLength = 0.01 
# for i in range(100000):
#     veloBRW_pts.append(generateUniformSpherePoint())
#     # veloBRW_pts.append(generateWeightedSpherePoint(1))

# veloBRW_prevPos = [0,0]
# veloBRW_pointsInRange = []
# veloBRW_coneLengthSquared = veloBRW_coneLength*veloBRW_coneLength #squared for performance
# veloBRW_coneMinLengthSquared = veloBRW_coneMinLength*veloBRW_coneMinLength #squared for performance

# veloBRW_old_xy = [0, 0]
# veloBRW_target_xy = [0, 0]
# def executeBoundedRandomVelocity(elapsed_time: float, desired_g: float, pos_sph: list):
#     global veloBRW_pts, veloBRW_maxVelocity, veloBRW_coneLengthSquared, veloBRW_coneMinLengthSquared, veloBRW_coneAngle, veloBRW_prevPos, veloBRW_lastChange, veloBRW_changeDT, veloBRW_pointsInRange, veloBRW_old_xy, veloBRW_target_xy

#     # This block now ONLY sets a new target velocity every second
#     if(elapsed_time > (veloBRW_lastChange + veloBRW_changeDT)):
#         veloBRW_pointsInRange = []
#         veloBRW_lastChange += veloBRW_changeDT
#         prev_pos_cart = sph2cart(*veloBRW_prevPos)
#         pos_cart = sph2cart(*pos_sph)

#         # Store the current velocity as the "old" velocity for the start of our interpolation
#         veloBRW_old_xy = veloBRW_target_xy

#         # velo = v3Scale(v3Sub(pos_cart, prev_pos_cart), 1 / (1/5.0))
#         velo = v3Scale(v3Sub(pos_cart, prev_pos_cart), 1 / veloBRW_changeDT)
#         for candidatePt in veloBRW_pts:
#             candVector = v3Sub(candidatePt, pos_cart)
#             magS = magSquared3(candVector)
#             if(magS < veloBRW_coneLengthSquared and magS > veloBRW_coneMinLengthSquared):
#                 if(magSquared3(velo) == 0 or v3Angle(candVector,velo) < veloBRW_coneAngle): # from startup
#                     veloBRW_pointsInRange.append(candidatePt)
#         if(len(veloBRW_pointsInRange) != 0):
#             pt = random.choice(veloBRW_pointsInRange)
#             veloBRW_pts.remove(pt)
#             veloBRW_pts.append(generateUniformSpherePoint())

#             # --- This is the new, correct spherical math from our previous discussion ---
#             cart_velo = norm3(v3Sub(pt, pos_cart))    # finding direct vector to the desired point
#             theta, phi = pos_sph[0], pos_sph[1]       # defining inner and outer theta

#             # defining local basis vectors that are tangent to sphere at current location
#             theta_hat = [-math.sin(theta), math.cos(theta), 0] # constant latitude vector:  partial derivative of the sph2cart conversion with respect to the azimuth angle (θ).
#             phi_hat = [math.cos(theta) * math.cos(phi), math.sin(theta) * math.cos(phi), -math.sin(phi)]  # constant longitude vector: partial derivative with respect to the elevation angle (ϕ).
            
#             # project 3D path onto 2D ground
#             velo_theta = v3Dot(cart_velo, theta_hat)
#             velo_phi = v3Dot(cart_velo, phi_hat)
            
#             # Scale so final speed is desired maximum velocity
#             veloBRW_target_xy = v2Scale([velo_theta, velo_phi], veloBRW_maxVelocity)
#         else:
#             print("WARN: No points found in cone. Maintaining Velocity.")
#             # If no points are found, just keep aiming for the current target
#             veloBRW_old_xy = veloBRW_target_xy

#     veloBRW_prevPos = pos_sph # save previous
    
#     # --- SMOOTHING LOGIC (runs on every call) ---
#     # 1. Calculate our progress through the current 1-second interval (from 0.0 to 1.0)
#     progress = (elapsed_time - veloBRW_lastChange) / veloBRW_changeDT
#     progress = max(0.0, min(1.0, progress)) # Clamp progress between 0 and 1

#     # 2. Linearly interpolate between the old and target velocities
#     x = veloBRW_old_xy[0] + (veloBRW_target_xy[0] - veloBRW_old_xy[0]) * progress
#     y = veloBRW_old_xy[1] + (veloBRW_target_xy[1] - veloBRW_old_xy[1]) * progress

#     return [x, y, veloBRW_pointsInRange]





# # dis is ass
# # --- Global Variables for Stable Bounded Random Velocity ---
# veloBRW_lastChange = 0
# veloBRW_changeDT = 1.0  # seconds
# veloBRW_pts = [generateUniformSpherePoint() for _ in range(100000)]
# veloBRW_maxVelocity = VELO_MAX
# veloBRW_coneAngle = 5 * math.pi / 180.0
# veloBRW_coneLength = 0.55
# veloBRW_coneMinLength = 0.01

# veloBRW_prevPos = [0, 0]
# veloBRW_pointsInRange = []
# veloBRW_coneLengthSquared = veloBRW_coneLength * veloBRW_coneLength
# veloBRW_coneMinLengthSquared = veloBRW_coneMinLength * veloBRW_coneMinLength

# # --- Add state variables for smoothing ---
# veloBRW_old_xy = [0, 0]      # Holds the velocity at the start of the interval
# veloBRW_target_xy = [0, 0]   # Holds the velocity at the end of the interval


# def executeBoundedRandomVelocityWEIRD(elapsed_time: float, desired_g: float, pos_sph: list, outer_theta: float):
#     global veloBRW_pts, veloBRW_maxVelocity, veloBRW_coneLengthSquared, veloBRW_coneMinLengthSquared, \
#            veloBRW_coneAngle, veloBRW_prevPos, veloBRW_lastChange, veloBRW_changeDT, \
#            veloBRW_pointsInRange, veloBRW_target_xy, veloBRW_old_xy # <-- Add old_xy

#     # --- Decision Block: Calculate a new target velocity once per interval ---
#     if elapsed_time > (veloBRW_lastChange + veloBRW_changeDT):
#         veloBRW_pointsInRange = []
#         veloBRW_lastChange += veloBRW_changeDT
        
#         # Store the current target as the "old" one for the start of the interpolation
#         veloBRW_old_xy = veloBRW_target_xy
        
#         prev_pos_cart = sph2cart(*veloBRW_prevPos)
#         pos_cart = sph2cart(*pos_sph)

#         # 1. Filter points into a forward-facing cone
#         velo_for_cone = v3Sub(pos_cart, prev_pos_cart)
#         for candidatePt in veloBRW_pts:
#             candVector = v3Sub(candidatePt, pos_cart)
#             magS = magSquared3(candVector)
#             if magS < veloBRW_coneLengthSquared and magS > veloBRW_coneMinLengthSquared:
#                 if magSquared3(velo_for_cone) == 0 or v3Angle(candVector, velo_for_cone) < veloBRW_coneAngle:
#                     veloBRW_pointsInRange.append(candidatePt)
        
#         # 2. Choose a random point from the valid cone
#         if len(veloBRW_pointsInRange) > 0:
#             pt = random.choice(veloBRW_pointsInRange)
#             veloBRW_pts.remove(pt)
#             veloBRW_pts.append(generateUniformSpherePoint())

#             # 3. Stable Kinematic Conversion
#             cart_velo = norm3(v3Sub(pt, pos_cart))
#             omega_total_3D = v3Cross(pos_cart, cart_velo)
            
#             desired_outer_omega = omega_total_3D[0]
#             inner_gimbal_axis = rotateX([0, 1, 0], outer_theta)
#             desired_inner_omega = v3Dot(omega_total_3D, inner_gimbal_axis)
            
#             mag = mag2([desired_outer_omega, desired_inner_omega])
#             if mag > 0:
#                 scale_factor = veloBRW_maxVelocity / mag
#                 veloBRW_target_xy = [desired_outer_omega * scale_factor, desired_inner_omega * scale_factor]
#             else:
#                 veloBRW_target_xy = [0, 0]
#         else:
#             print("WARN: No points found in cone. Maintaining Velocity.")
#             # If no new point is found, ensure no interpolation happens by setting old = new
#             veloBRW_old_xy = veloBRW_target_xy

#     veloBRW_prevPos = pos_sph
    
#     # --- Smoothing Logic (runs on every call) ---
#     # 1. Calculate progress through the current interval (from 0.0 to 1.0)
#     progress = (elapsed_time - veloBRW_lastChange) / veloBRW_changeDT
#     progress = max(0.0, min(1.0, progress)) # Clamp progress to prevent overshooting

#     # 2. Linearly interpolate between the old and new target velocities
#     x = veloBRW_old_xy[0] + (veloBRW_target_xy[0] - veloBRW_old_xy[0]) * progress
#     y = veloBRW_old_xy[1] + (veloBRW_target_xy[1] - veloBRW_old_xy[1]) * progress

#     return [x, y, veloBRW_pointsInRange]


posBRW_lastChange = 0
posBRW_changeDT = 0.03 #sec
posBRW_pts = []
posBRW_maxVelocity = VELO_MAX
posBRW_coneAngle = 5 * math.pi / 180 # Radial angle of the cone in radians (Half of max angle of cone)
posBRW_coneLength = 0.55
posBRW_coneMinLength = 0.1 
for i in range(100000):
    posBRW_pts.append(generateUniformSpherePoint())
    #veloBRW_pts.append(generateWeightedSpherePoint(1))

posBRW_prevPos = [0,0]
posBRW_pointsInRange = []
posBRW_coneLengthSquared = posBRW_coneLength*posBRW_coneLength #squared for performance
posBRW_coneMinLengthSquared = posBRW_coneMinLength*posBRW_coneMinLength #squared for performance
def executeBoundedRandomPosition(elapsed_time: float, desired_g: float, pos_sph: list):
    global posBRW_pts, posBRW_maxVelocity, posBRW_coneLengthSquared, posBRW_coneMinLengthSquared, posBRW_coneAngle, posBRW_prevPos, posBRW_lastChange, posBRW_changeDT, x, y, posBRW_pointsInRange
    # pos_sph = [fmod(pos_sph[0], 2*math.pi), fmod(pos_sph[1], 2*math.pi)]

    if(elapsed_time > (posBRW_lastChange + posBRW_changeDT)):
        print("\n--- NEW POSBRW CALCULATION ---")
        posBRW_pointsInRange = []
        posBRW_lastChange += posBRW_changeDT
        
        print(f"  1. PREV SPH: az={posBRW_prevPos[0]:.3f}, el={posBRW_prevPos[1]:.3f}")
        print(f"  1. CURR SPH: az={pos_sph[0]:.3f}, el={pos_sph[1]:.3f}")

        prev_pos_cart = sph2cart(posBRW_prevPos[1], posBRW_prevPos[0])
        pos_cart = sph2cart(pos_sph[1], pos_sph[0])
        
        print(f"  2. PREV CART: x={prev_pos_cart[0]:.3f}, y={prev_pos_cart[1]:.3f}, z={prev_pos_cart[2]:.3f}")
        print(f"  2. CURR CART: x={pos_cart[0]:.3f}, y={pos_cart[1]:.3f}, z={pos_cart[2]:.3f}")

        # This is the vector that defines the cone's direction
        pos_velo = v3Sub(pos_cart, prev_pos_cart)
        print(f"  3. CONE DIR (unscaled): x={pos_velo[0]:.3f}, y={pos_velo[1]:.3f}, z={pos_velo[2]:.3f}")

        if magSquared3(pos_velo) == 0:
            print("  3a. WARN: Zero velocity, cone will be fully open.")

        for candidatePt in posBRW_pts:
            candVector = v3Sub(candidatePt, pos_cart)
            magS = magSquared3(candVector)
            if(magS < posBRW_coneLengthSquared and magS > posBRW_coneMinLengthSquared):
                if(magSquared3(pos_velo) == 0 or v3Angle(candVector, pos_velo) < posBRW_coneAngle):
                    posBRW_pointsInRange.append(candidatePt)
        
        print(f"  4. Found {len(posBRW_pointsInRange)} points in cone.")

        if(len(posBRW_pointsInRange) != 0):
            pt = random.choice(posBRW_pointsInRange)
            print(f"  5. CHOSEN CART: x={pt[0]:.3f}, y={pt[1]:.3f}, z={pt[2]:.3f}")
            
            desired = cart2sph(pt[0], pt[1], pt[2])
            posBRW_pts.remove(pt)
            posBRW_pts.append(generateUniformSpherePoint())
            
            print(f"  6. DESIRED SPH: az={desired[0]:.3f}, el={desired[1]:.3f}")
            x, y = desired
        else:
            print("  WARN: No points found in cone. Maintaining previous target.")
            # If no points, we don't change x and y, so the target remains the same.
    
    posBRW_prevPos = pos_sph
    return [x,y,posBRW_pointsInRange]



        # else:
        #     # FALLBACK: If the cone is empty, pick any random point to get unstuck
        #     print("WARN: No points found in cone. Picking random point to unstick.")
        #     pt = random.choice(posBRW_pts) # Choose from the whole list
        #     desired = cart2sph(*pt)
        #     posBRW_pts.remove(pt)
        #     posBRW_pts.append(generateUniformSpherePoint())
        #     x, y = desired
