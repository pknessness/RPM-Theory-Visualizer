import math, random

#CONSTANTS
sample_frequency = 100 #Hz
dt = 1/sample_frequency #seconds
speed_mult = 0 

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

#VectorBox
VECTOR_BOX = 1

#profile (rad/s)

# VELO_MAX = 0.785
# VELO_MAX = 1.57
VELO_MAX = 3.14159

#0.733 rads = 7RPM
#0.785 rads = 7.5RPM
#1.571 rads = 15RPM
#3.142 rads = 30RPM

def rand_seed(seed: int):
    random.seed(seed)

def mag2(vector: list):
    return math.sqrt(vector[0]*vector[0] + vector[1]*vector[1])

def mag3(vector: list):
    return math.sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2])

def magSquared3(vector: list):
    return vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]

def norm2(vec: list):
    magn = mag2(vec)
    return [vec[0]/magn, vec[1]/magn]

def norm3(vec: list):
    magn = mag3(vec)
    return [vec[0]/magn, vec[1]/magn, vec[2]/magn]

# def sph2cart(theta: float, phi: float):
#     #theta azimuth, phi elevation
#     #print([theta, phi], math.sin(phi), math.cos(phi), math.sin(theta), math.cos(theta))
#     x1 = math.cos(phi) * math.cos(theta)
#     y1 = math.cos(phi) * math.sin(theta)
#     z1 = math.sin(phi)
#     #print([theta, phi], [x1, y1, z1])
#     return [x1, y1, z1]

# def cart2sph(x: float, y: float, z: float):
#     #theta azimuth, phi elevation
#     #theta = math.acos(z)
#     #phi = math.acos(x / math.sqrt(x*x + y*y))
#     theta = math.atan2(y, x)
#     phi = math.atan2(z, math.sqrt(x*x + y*y))
#     return [theta, phi]

def sph2cart(phi: float, theta: float):
    #theta azimuth, phi elevation
    #print([theta, phi], math.sin(phi), math.cos(phi), math.sin(theta), math.cos(theta))
    x1 = math.cos(2 * math.pi - phi) * math.cos(theta)
    y1 = math.cos(2 * math.pi - phi) * math.sin(theta)
    z1 = math.sin(2 * math.pi - phi)
    #print([theta, phi], [x1, y1, z1])
    return [z1, x1, y1]

def cart2sph(z: float, x: float, y: float):
    #theta azimuth, phi elevation
    #theta = math.acos(z)
    #phi = math.acos(x / math.sqrt(x*x + y*y))
    theta = math.atan2(y, x)
    phi = math.atan2(z, math.sqrt(x*x + y*y))
    return [2 * math.pi - phi, theta]

#phi theta
# x y z NO
# x z y kinda (inner good, outer not)
# y x z NO
# y z x NO
# z x y YES but inner flipped
# z y x NO

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

def fmodl(num: list, base: float):
    res = []
    for op in num:
        while(op > base):
            op -= base
        while(op < 0):
            op += base
        res.append(op)
    return res

def delta(source: float, destination: float, mod: float):
    delta = destination - source
    if(math.fabs(delta) > mod/2):
        if(delta > 0):
            delta -= mod
        else:
            delta += mod
    return delta

def generateUniformSpherePoint():
    while(1):
        point = [random.random() * 2 - 1, random.random() * 2 - 1, random.random() * 2 - 1]
        if(mag3(point) < 1):
            return norm3(point)
        
def generateUniformSpherePoint2():
    theta = 2 * math.pi * random.random() #azimuth
    phi = math.acos(1 - 2 * random.random()) #elevation
    return sph2cart(theta, phi)

f = open("distribution.csv", "w")
f.close()
f = open("distrib_x.csv", "w")
f.close()
f = open("distrib_y.csv", "w")
f.close()
f = open("distrib_z.csv", "w")
f.close()
f = open("after_x.csv", "w")
f.close()
f = open("after_y.csv", "w")
f.close()
f = open("after_z.csv", "w")
f.close()
# f = open("distribution.csv", "w")
# f.close()
def generateWeightedSpherePoint(weight: float):
    while(1):
        which = random.random()
        y = 0
        if(which < weight):
            y = math.sqrt(random.random())
        else:
            y = random.random()
        point = [random.random() * 2 - 1, y * 2 - 1, random.random() * 2 - 1]
        # f = open("distrib_x.csv", "a")
        # f.write(f"{point[0]}\n")
        # f.close()
        # f = open("distrib_y.csv", "a")
        # f.write(f"{point[1]}\n")
        # f.close()
        # f = open("distrib_z.csv", "a")
        # f.write(f"{point[2]}\n")
        # f.close()
        if(mag3(point) < 1):
            # f = open("after_x.csv", "a")
            # f.write(f"{point[0]}\n")
            # f.close()
            # f = open("after_y.csv", "a")
            # f.write(f"{point[1]}\n")
            # f.close()
            # f = open("after_z.csv", "a")
            # f.write(f"{point[2]}\n")
            # f.close()
            return norm3(point)
        
def generateCosWeightedSpherePoint(weight: float):
    while(1):
        which = random.random()
        y = 0
        if(which < weight):
            y = math.cos(random.random()) * 2 - 1
        else:
            y = random.random()
        point = [random.random() * 2 - 1, y * 2 - 1, random.random() * 2 - 1]
        # f = open("distrib_x.csv", "a")
        # f.write(f"{point[0]}\n")
        # f.close()
        # f = open("distrib_y.csv", "a")
        # f.write(f"{point[1]}\n")
        # f.close()
        # f = open("distrib_z.csv", "a")
        # f.write(f"{point[2]}\n")
        # f.close()
        if(mag3(point) < 1):
            f = open("after_x.csv", "a")
            f.write(f"{point[0]}\n")
            f.close()
            f = open("after_y.csv", "a")
            f.write(f"{point[1]}\n")
            f.close()
            f = open("after_z.csv", "a")
            f.write(f"{point[2]}\n")
            f.close()
            return norm3(point)

def generateLinearWeightedSpherePoint(weight: float):
    theta = 2 * math.pi * random.random() #azimuth
    phi = math.acos(1 - 2 * random.random()) #elevation
    if(random.random() < weight):
        phi = math.sqrt(phi / math.pi) * math.pi
    return sph2cart(theta, phi)

def fsin(number):
    return math.sin(number)

def fcos(number):
    return math.cos(number)

def v3Add(a: list, b: list):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]

def v3Sub(a: list, b: list):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]

def v2Sub(a: list, b: list):
    return [a[0] - b[0], a[1] - b[1]]

def v3Scale(a: list, scalar: float):
    return [a[0] * scalar, a[1] * scalar, a[2] * scalar]

def v2Scale(a: list, scalar: float):
    return [a[0] * scalar, a[1] * scalar]

def v3Angle(a: list, b: list):
    A = norm3(a)
    B = norm3(b)
    dotP = A[0] * B[0] + A[1] * B[1] + A[2] * B[2]
    return math.acos(dotP)