import numpy as np
import math
import sys

import pygame
from pygame.locals import *

from constants import *

# height = 0
# width = 0
aspect = 16.0/9.0
fov = 100
projectionPanelHeight = 80
projectionPanelPos = np.array([0,-20,10])
zoomFactor = 1
cameraDir = projectionPanelPos * -1 #[0,5,2]
cameraSideways = np.array([0,0,1]) #can be undefined and will autogen

vertices = []
squares = []
quads = []

backgroundColor = "#FFFFFF"
frameColor = "#000000"
vectorColorX = "#E85F2A"
vectorColorY = "#5FE82A"
vectorColorZ = "#2A5FE8"

vectorColors = ["#E85F2A", "#5FE82A", "#2A5FE8", "#9C27B0", "#F48FB1", "#FFB74D", "#00BCD4"]

class vec3D:
    def __init__(self, dir: list, src = [0,0,0], scalar = 1, solid = False):
        self.src = src
        self.dir = [dir[0]*scalar, dir[1]*scalar, dir[2]*scalar]
        self.solid = solid

fps = 60
start_width, start_height = 640, 480

basic_font, fpsClock, screen = None, None, None

def init():
    global fps, start_height, start_width, basic_font, fpsClock, screen
    pygame.init()

    pygame.font.init() # you have to call this at the start, 
    basic_font = pygame.font.SysFont('Agency FB', 30)

    fpsClock = pygame.time.Clock()

    screen = pygame.display.set_mode((start_width, start_height), pygame.RESIZABLE)

class Quad:
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        
    def __str__(self):
        return f"[[{self.a[0]:.2f},{self.a[1]:.2f}],[{self.b[0]:.2f},{self.b[1]:.2f}],[{self.c[0]:.2f},{self.c[1]:.2f}],[{self.d[0]:.2f},{self.d[1]:.2f}]]"

def rotateTranslateX(vertex):
    theta = math.atan2(cameraDir[1],cameraDir[2])
    #theta = 0
    v2x = vertex[0] - projectionPanelPos[0]*zoomFactor
    v2y = vertex[1] - projectionPanelPos[1]*zoomFactor
    v2z = vertex[2] - projectionPanelPos[2]*zoomFactor
    cs = fcos(theta)
    sn = fsin(theta)
    v2 = [v2x, v2y * cs - v2z * sn, v2y * sn + v2z * cs]
    return v2

def rotateTranslateY(vertex):
    theta = math.atan2(cameraDir[2],cameraDir[0])
    # print("theta",theta)
    v2 = [
        vertex[0] - projectionPanelPos[0]*zoomFactor,
        vertex[1] - projectionPanelPos[1]*zoomFactor,
        vertex[2] - projectionPanelPos[2]*zoomFactor]
    # print("v2",v2)
    cs = fcos(theta)
    sn = fsin(theta)
    #v2 = [v2[0], v2[1] * cs - v2[2] * sn, v2[1] * sn + v2[2] * cs]
    v2 = [v2[0] * cs - v2[2] * sn, v2[1], v2[0] * sn + v2[2] * cs]
    # print("v2",v2)
    return v2

def rotateTranslateZ(vertex):
    theta = math.atan2(cameraDir[1],cameraDir[0])
    # print("theta",theta)
    v2 = [
        vertex[0] - projectionPanelPos[0]*zoomFactor,
        vertex[1] - projectionPanelPos[1]*zoomFactor,
        vertex[2] - projectionPanelPos[2]*zoomFactor]
    # print("v2",v2)
    cs = fcos(theta)
    sn = fsin(theta)
    #v2 = [v2[0], v2[1] * cs - v2[2] * sn, v2[1] * sn + v2[2] * cs]
    v2 = [v2[0] * cs - v2[1] * sn, v2[0] * sn + v2[1] * cs, v2[2]]
    # print("v2",v2)
    return v2

def rotateX(vertex, angle):
    cs = fcos(angle)
    sn = fsin(angle)
    v2 = [vertex[0], vertex[1] * cs - vertex[2] * sn, vertex[1] * sn + vertex[2] * cs]
    return v2

def rotateY(vertex, angle):
    cs = fcos(angle)
    sn = fsin(angle)
    v2 = [vertex[0] * cs - vertex[2] * sn, vertex[1],  vertex[0] * sn + vertex[2] * cs]
    return v2

def rotateZ(vertex, angle):
    cs = fcos(angle)
    sn = fsin(angle)
    v2 = [vertex[0] * cs - vertex[1] * sn, vertex[0] * sn + vertex[1] * cs, vertex[2]]
    return v2

def strokeQuad(quad: Quad, scale: float, pixelOffset, color = "#FFFFFF", lineWidth = 2):
    pygame.draw.polygon(screen, pygame.Color(color), [(quad.a[0] * scale + pixelOffset[0], quad.a[1] * scale + pixelOffset[1]), (quad.b[0] * scale + pixelOffset[0], quad.b[1] * scale + pixelOffset[1]), (quad.c[0] * scale + pixelOffset[0], quad.c[1] * scale + pixelOffset[1]), (quad.d[0] * scale + pixelOffset[0], quad.d[1] * scale + pixelOffset[1])], lineWidth)

def createRect(center,x,y,z):
    addSquare(
        [center[0] + x/2,center[1] + y/2,center[2] + z/2],
        [center[0] + x/2,center[1] + y/2,center[2] - z/2],
        [center[0] + x/2,center[1] - y/2,center[2] - z/2],
        [center[0] + x/2,center[1] - y/2,center[2] + z/2])
    
    addSquare(
        [center[0] - x/2,center[1] + y/2,center[2] + z/2],
        [center[0] - x/2,center[1] + y/2,center[2] - z/2],
        [center[0] - x/2,center[1] - y/2,center[2] - z/2],
        [center[0] - x/2,center[1] - y/2,center[2] + z/2])
    addSquare(
        [center[0] + x/2,center[1] - y/2,center[2] + z/2],
        [center[0] + x/2,center[1] - y/2,center[2] - z/2],
        [center[0] - x/2,center[1] - y/2,center[2] - z/2],
        [center[0] - x/2,center[1] - y/2,center[2] + z/2])
    
    addSquare(
        [center[0] + x/2,center[1] + y/2,center[2] + z/2],
        [center[0] + x/2,center[1] + y/2,center[2] - z/2],
        [center[0] - x/2,center[1] + y/2,center[2] - z/2],
        [center[0] - x/2,center[1] + y/2,center[2] + z/2])
    addSquare(
        [center[0] + x/2,center[1] + y/2,center[2] + z/2],
        [center[0] + x/2,center[1] - y/2,center[2] + z/2],
        [center[0] - x/2,center[1] - y/2,center[2] + z/2],
        [center[0] - x/2,center[1] + y/2,center[2] + z/2])
    
    addSquare(
        [center[0] + x/2,center[1] + y/2,center[2] - z/2],
        [center[0] + x/2,center[1] - y/2,center[2] - z/2],
        [center[0] - x/2,center[1] - y/2,center[2] - z/2],
        [center[0] - x/2,center[1] + y/2,center[2] - z/2])


def createCube(center,s):
    createRect(center,s,s,s)

def pointToProjected(vertex):
    if(len(vertex) != 3): print(f"pTP Error: Vertex {vertex} is not the right size")
    vert = rotateTranslateX(vertex)
    # vert2 = rotateTranslateY(vert1)
    # vert2 = vert1
    # vert = rotateTranslateZ(vert2)
    # vert = vert2
    px = (fov * vert[0])/(fov + vert[2])
    py = (fov * vert[1])/(fov + vert[2])
    return [px, py]

def squareToQuad(square):
    # console.log(vertices[square[0]])
    #q = {a:pointToProjected(vertices[square[0]]),b:pointToProjected(vertices[square[1]]),c:pointToProjected(vertices[square[2]]),d:pointToProjected(vertices[square[3]])}
    q = Quad(
        pointToProjected(vertices[square[0]]),
        pointToProjected(vertices[square[1]]),
        pointToProjected(vertices[square[2]]),
        pointToProjected(vertices[square[3]]))
    quads.append(q)
    return q

def addSquare(e, f, g, h):
    square = [-1,-1,-1,-1]
    
    square[0] = len(vertices)
    vertices.append(e)
    square[1] = len(vertices)
    vertices.append(f)
    square[2] = len(vertices)
    vertices.append(g)
    square[3] = len(vertices)
    vertices.append(h)
    
    squares.append(square)

def drawing(color: str = "#FFFFFF", reset = True, solid = False):
    global vertices, squares, quads
    for square in squares:
        q = squareToQuad(square)
        # console.log(squares[i])
        # console.log(q)
        # fillQuad(q,height/projectionPanelHeight,[width/2,height/2])
        w, h = pygame.display.get_window_size()
        strokeQuad(q,h/projectionPanelHeight,[w/2,h/2], color, 1 if (not solid) else 0)
        # print(q,height/projectionPanelHeight,[width/2,height/2])
    if(reset):
        vertices = []
        squares = []
        quads = []

def addOuter():
    createRect([0,0,0],OUTER_SIDE_LENGTH,FRAME_THICKNESS,OUTER_SIDE_LENGTH)
    createRect([0,0,0],OUTER_SIDE_LENGTH-2*FRAME_THICKNESS,FRAME_THICKNESS,OUTER_SIDE_LENGTH-2*FRAME_THICKNESS)

def addInner():
    createRect([0,0,0],INNER_SIDE_LENGTH,FRAME_THICKNESS,INNER_SIDE_LENGTH)
    createRect([0,0,0],INNER_SIDE_LENGTH-2*FRAME_THICKNESS,FRAME_THICKNESS,INNER_SIDE_LENGTH-2*FRAME_THICKNESS)
    createRect([0,0,FRAME_THICKNESS/6],PLATFORM_SIDE_LENGTH,PLATFORM_SIDE_LENGTH,FRAME_THICKNESS/3)

def addFrame():
    outest = OUTER_SIDE_LENGTH + 2*FRAME_THICKNESS
    createRect([0,0,-outest/2 + FRAME_THICKNESS/2],outest,outest,FRAME_THICKNESS)
    createRect([0,0,-outest/2 + FRAME_THICKNESS/2],outest-2*FRAME_THICKNESS,outest-2*FRAME_THICKNESS,FRAME_THICKNESS)
    
    createRect([outest/2 - FRAME_THICKNESS/2,0,-outest/4 + FRAME_THICKNESS],FRAME_THICKNESS,FRAME_THICKNESS,outest/2)
    createRect([-outest/2 + FRAME_THICKNESS/2,0,-outest/4 + FRAME_THICKNESS],FRAME_THICKNESS,FRAME_THICKNESS,outest/2)
    #createRect([0,0,0],INNER_SIDE_LENGTH-2*FRAME_THICKNESS,FRAME_THICKNESS,INNER_SIDE_LENGTH-2*FRAME_THICKNESS)


def setup2():
    createRect([0,0,0],15,15,15)
    createRect([0,0,0],2,2,2)

def createWireframeSphere(center, radius, lat_lines=12, lon_lines=16):
    """Create a wireframe sphere using latitude and longitude lines"""
    import math
    
    # Draw latitude lines (circles parallel to equator)
    for i in range(lat_lines):
        phi = math.pi * (i / lat_lines) - math.pi/2  # from -pi/2 to pi/2
        r = radius * math.cos(phi)  # radius of this latitude circle
        z = radius * math.sin(phi)  # height of this latitude circle
        
        # Create circle at this latitude
        points = []
        segments = lon_lines * 2
        for j in range(segments + 1):
            theta = 2 * math.pi * j / segments
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            points.append([center[0] + x, center[1] + y, center[2] + z])
        
        # Connect consecutive points
        for j in range(len(points) - 1):
            addSquare(points[j], points[j+1], points[j+1], points[j])
    
    # Draw longitude lines (circles through poles)
    for i in range(lon_lines):
        theta = 2 * math.pi * i / lon_lines
        
        # Create semicircle through poles
        points = []
        segments = lat_lines * 2
        for j in range(segments + 1):
            phi = math.pi * j / segments - math.pi/2  # from -pi/2 to pi/2
            x = radius * math.cos(phi) * math.cos(theta)
            y = radius * math.cos(phi) * math.sin(theta)
            z = radius * math.sin(phi)
            points.append([center[0] + x, center[1] + y, center[2] + z])
        
        # Connect consecutive points
        for j in range(len(points) - 1):
            addSquare(points[j], points[j+1], points[j+1], points[j])

angle = 0
renderPointsIndex = 0
renderPointsPer = 500
look_trail = []  # Stores the path history of the look vector
max_trail_length = 500  # Maximum number of trail points to keep

def render(actual: list, desired: list, vectors: list[vec3D], text: list, points: list):
    global x, cameraDir, projectionPanelPos, angle, fov, zoomFactor, renderPointsIndex, renderPointsPer, look_trail
    global vertices, squares, quads
    
    screen.fill(backgroundColor)
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == MOUSEWHEEL:
            # fov += event.y * 10
            zoomFactor -= event.y / 10
            if(zoomFactor < -2.4):
                zoomFactor = -2.4
            elif(zoomFactor > 2):
                zoomFactor = 2
    
    move_speed = 1
    rotate_speed = 0.01
    
    keys=pygame.key.get_pressed()
    if keys[K_w]:
        projectionPanelPos += [0,move_speed,0]
        if(projectionPanelPos[1] > -1):
            projectionPanelPos[1] = -1
        cameraDir = projectionPanelPos * -1
    if keys[K_s]:
        projectionPanelPos += [0,-move_speed,0]
        if(projectionPanelPos[1] < -150):
            projectionPanelPos[1] = -150
        cameraDir = projectionPanelPos * -1
    if keys[K_a]:
        angle += rotate_speed
    if keys[K_d]:
        angle -= rotate_speed
    if keys[K_e]:
        renderPointsPer = 100000
    else:
        renderPointsPer = 500
    if keys[K_c]:
        look_trail.clear()  # Clear the trail when C is pressed
    
    if(type(desired) == list):
        # Update DESIRED
        vertices = []
        squares = []
        quads = []
        addInner()
        
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], desired[1])
            
        addOuter()
        
        for i in range(len(vertices)):
            vertices[i] = rotateX(vertices[i], desired[0])
        
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], angle)
        
        # Draw DESIRED
        drawing("#BEF300")
    
    # Update ACTUAL
    # vertices = []
    # squares = []
    # quads = []
    
    addInner()
    
    for i in range(len(vertices)):
        vertices[i] = rotateZ(vertices[i], actual[1])
        
    addOuter()
    
    for i in range(len(vertices)):
        vertices[i] = rotateX(vertices[i], actual[0])
    
    addFrame()
    
    for i in range(len(vertices)):
        vertices[i] = rotateZ(vertices[i], angle)
    
    # Draw ACTUAL
    drawing(frameColor)
    
    # Draw Vectors
    # vertices = []
    # squares = []
    # quads = []
    
    cnt = 0
    for v in vectors:
        xoffset, yoffset, zoffset = 0, 0, 0
        if(v.dir[0] > 0): 
            xoffset = VECTOR_BOX/2
        elif(v.dir[0] < 0):
            xoffset = -VECTOR_BOX/2
            
        if(v.dir[2] > 0): 
            yoffset = VECTOR_BOX/2
        elif(v.dir[2] < 0):
            yoffset = -VECTOR_BOX/2
            
        if(v.dir[1] > 0): 
            zoffset = VECTOR_BOX/2
        elif(v.dir[1] < 0):
            zoffset = -VECTOR_BOX/2
        
        createRect([v.dir[0]/2 + v.src[0] + xoffset, v.src[2] + 0, v.src[1] + 0], v.dir[0], VECTOR_BOX, VECTOR_BOX)
        
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], actual[1])
        for i in range(len(vertices)):
            vertices[i] = rotateX(vertices[i], actual[0])
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], angle)
        
        drawing(vectorColors[0], True, v.solid)
        
        createRect([v.src[0] + 0,v.dir[2]/2 + v.src[2] + yoffset, v.src[1] + 0], VECTOR_BOX, v.dir[2], VECTOR_BOX)
        
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], actual[1])
        for i in range(len(vertices)):
            vertices[i] = rotateX(vertices[i], actual[0])
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], angle)
        
        drawing(vectorColors[1], True, v.solid)
        
        createRect([ v.src[0] + 0, v.src[2] + 0,v.dir[1]/2 + v.src[1] + zoffset], VECTOR_BOX, VECTOR_BOX, v.dir[1])
        
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], actual[1])
        for i in range(len(vertices)):
            vertices[i] = rotateX(vertices[i], actual[0])
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], angle)
        
        drawing(vectorColors[2], True, v.solid)
        
        createCube([v.src[0],v.src[2],v.src[1]], VECTOR_BOX)
        
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], actual[1])
        for i in range(len(vertices)):
            vertices[i] = rotateX(vertices[i], actual[0])
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], angle)
        
        drawing(frameColor, True, v.solid)
    
    
    # print(zoomFactor)
    cloudR = 55
    
    # Draw 'look' vector: where the RPM is pointing on the unit sphere
    # The gravity vector points down in the RPM's local frame (0, -1, 0)
    # After rotations, it shows where on the sphere the RPM is currently "looking"
    # This matches the point cloud cone logic in rpm_profile.py
    from constants import sph2cart
    
    # Convert the actual position (inner_theta=azimuth, outer_theta=elevation) to Cartesian
    # actual[1] = inner_theta, actual[0] = outer_theta
    look_cart = sph2cart(actual[1], actual[0])
    
    # Add current look position to trail (before rotation)
    look_trail.append(look_cart.copy())
    if len(look_trail) > max_trail_length:
        look_trail.pop(0)  # Remove oldest point
    
    # Apply scene rotation for visualization
    look = rotateZ(look_cart, angle)

    # place the look point on the unit sphere scaled to cloudR (same radius used for points)
    look_point_world = [look[0] * cloudR, look[1] * cloudR, look[2] * cloudR]

    # project to screen
    w, h = pygame.display.get_window_size()
    scale = h/projectionPanelHeight
    pt = pointToProjected(look_point_world)

    # draw a line from origin to look point and a small filled circle at the tip
    origin_projected = pointToProjected([0,0,0])
    pygame.draw.line(screen, (255, 0, 0), (origin_projected[0] * scale + w/2, origin_projected[1] * scale + h/2), (pt[0] * scale + w/2, pt[1] * scale + h/2), 2)
    pygame.draw.circle(screen, (255, 0, 0), (int(pt[0] * scale + w/2), int(pt[1] * scale + h/2)), 5)
    
    # Draw the trail path on the sphere surface
    if len(look_trail) > 1:
        trail_points_2d = []
        for trail_pt in look_trail:
            rotated_trail = rotateZ(trail_pt, angle)
            trail_world = [rotated_trail[0] * cloudR, rotated_trail[1] * cloudR, rotated_trail[2] * cloudR]
            trail_2d = pointToProjected(trail_world)
            trail_points_2d.append([trail_2d[0] * scale + w/2, trail_2d[1] * scale + h/2])
        
        # Draw the trail as connected lines with gradient color (older = more transparent)
        for i in range(len(trail_points_2d) - 1):
            # Calculate alpha based on position in trail (newer = more opaque)
            alpha = int(100 + (155 * i / len(trail_points_2d)))  # 100 to 255
            trail_color = (255, 100, 100, alpha)  # Reddish trail
            pygame.draw.line(screen, trail_color, trail_points_2d[i], trail_points_2d[i+1], 2)
    
    # Draw translucent wireframe sphere - optimized version using direct line drawing
    sphere_color = (170, 170, 170, 100)  # Light gray with some transparency
    lat_lines = 6
    lon_lines = 8
    
    import math
    # Draw latitude lines
    for i in range(1, lat_lines):  # Skip poles
        phi = math.pi * (i / lat_lines) - math.pi/2
        r = cloudR * math.cos(phi)
        z = cloudR * math.sin(phi)
        
        points_2d = []
        segments = lon_lines * 2
        for j in range(segments + 1):
            theta = 2 * math.pi * j / segments
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            pt_3d = rotateZ([x, y, z], angle)
            pt_2d = pointToProjected(pt_3d)
            points_2d.append([pt_2d[0] * scale + w/2, pt_2d[1] * scale + h/2])
        
        for j in range(len(points_2d) - 1):
            pygame.draw.line(screen, sphere_color, points_2d[j], points_2d[j+1], 1)
    
    # Draw longitude lines
    for i in range(lon_lines):
        theta = 2 * math.pi * i / lon_lines
        points_2d = []
        segments = lat_lines * 2
        for j in range(segments + 1):
            phi = math.pi * j / segments - math.pi/2
            x = cloudR * math.cos(phi) * math.cos(theta)
            y = cloudR * math.cos(phi) * math.sin(theta)
            z = cloudR * math.sin(phi)
            pt_3d = rotateZ([x, y, z], angle)
            pt_2d = pointToProjected(pt_3d)
            points_2d.append([pt_2d[0] * scale + w/2, pt_2d[1] * scale + h/2])
        
        for j in range(len(points_2d) - 1):
            pygame.draw.line(screen, sphere_color, points_2d[j], points_2d[j+1], 1)
    
    ind = 0
    while(renderPointsIndex < len(points) and ind < renderPointsPer):
        point = points[renderPointsIndex]
        # createCube([point[0]*cloudR, point[1]*cloudR, point[2]*cloudR], 1)
        w, h = pygame.display.get_window_size()
        scale = h/projectionPanelHeight
        point = rotateZ(point, angle)
        pt = pointToProjected([point[0]*cloudR, point[1]*cloudR, point[2]*cloudR])
        pygame.draw.circle(screen, vectorColors[3], [pt[0] * scale + w/2, pt[1] * scale + h/2], 3, 1)
        ind += 1
        renderPointsIndex += 1
    
    if(renderPointsIndex >= len(points)):
        renderPointsIndex = 0
    
    writeHeight = 0
    for line in text:
        text_surface = basic_font.render(line.text, False, line.color)
        screen.blit(text_surface, (0,writeHeight))
        writeHeight += 40
    
    pygame.display.flip()
    fpsClock.tick(fps)

x_omega_prev = 0
y_omega_prev = 0   
x_prev = 0
y_prev = 0   
x = 0
y = 0

MANUAL_POS_SCALING = 0.01
R_WORST_CASE = 0.21 #6 inch is 0.15m

ticks = 0

if __name__ == "__main__":
    instaccel = [0.0,0.0,0.0]
    acce = [0.0,0.0,0.0]
    
    prevMouse = [False, False, False]
    dragPos = [[0,0],[0,0],[0,0]]
    
    init()
    
    testPts = []
    for i in range(5000):
        testPts.append(generateUniformSpherePoint2())
        #testPts.append(generateLinearWeightedSpherePoint(1))
    
    while(1):
        mouse = pygame.mouse.get_pressed(num_buttons=3)
        
        renderings = [
            WriteLine(f"OUTER ANGLE: {x:.2f}/{fmod(x,2*math.pi):.2f} rad", (200,55,25)),
            WriteLine(f"INNER ANGLE: {y:.2f}/{fmod(y,2*math.pi):.2f} rad", (55,200,25))]
        
        renderings.append(WriteLine(f"INST: X:{instaccel[0]:.2f} Y:{instaccel[1]:.2f} Z:{instaccel[2]:.2f}", (205,100,200)))
        renderings.append(WriteLine(f"ProjectionPanelY:{projectionPanelPos[1]:.2f} Angle:{angle:.2f}", (205,100,200)))
        
        
        disp = [0,10,0]
        
        render([x,y], None, [vec3D(instaccel, disp), vec3D(acce, disp, 1, True)], renderings, testPts)
        #x += 0.013291
        #y += 0.23129
        #x += fsin(ticks/10)/100
        
        mouse_pos = pygame.mouse.get_pos()
        if(mouse[0] and prevMouse[0]):
            disp = [mouse_pos[0] - dragPos[0][0], mouse_pos[1] - dragPos[0][1]]
            x += disp[1] * MANUAL_POS_SCALING
            y += disp[0] * MANUAL_POS_SCALING
        if(mouse[0]):
            dragPos[0] = mouse_pos
        
        
        omega_x = (x - x_prev) / dt
        omega_y = (y - y_prev) / dt
        
        alpha_x = (omega_x - x_omega_prev) / dt
        alpha_y = (omega_y - y_omega_prev) / dt
        
        instaccel = [0,-9.81,0]
        
        instaccel = rotateX(instaccel, x)
        instaccel = rotateY(instaccel, -y)
        
        acce = [0,-3,0]
        
        acce = rotateX(acce, x)
        acce = rotateY(acce, -y)
        
        prevMouse = mouse
        x_prev = x
        y_prev = y
        x_omega_prev = omega_x
        y_omega_prev = omega_y
        
        ticks += 1
#xx nope
#xy bias +x
#yx
#yy
#zx
#zy
#x-x
#x-y
#y-x
#y-y
#z-x
#z-y