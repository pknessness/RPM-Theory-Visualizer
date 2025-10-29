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

# 0 1 0
# 0 cos(angle) sin(angle) (X)

# 0 1 0
# -sin(angle) cos(angle) 0 (Z)
# -sin(angle) cos(angle)cos(angle2) cos(angle)sin(angle2) (ZX)

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

angle = 0
renderPointsIndex = 0
renderPointsPer = 500

def render(actual: list, desired: list, vectors: list[vec3D], text: list, points: list, trail: list):
    global x, cameraDir, projectionPanelPos, angle, fov, zoomFactor, renderPointsIndex, renderPointsPer
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
    ind = 0
    w, h = pygame.display.get_window_size()
    scale = h/projectionPanelHeight
    while(renderPointsIndex < len(points) and ind < renderPointsPer):
        point = points[renderPointsIndex]
        # createCube([point[0]*cloudR, point[1]*cloudR, point[2]*cloudR], 1)
        point = rotateZ(point, angle)
        pt = pointToProjected([point[0]*cloudR, point[1]*cloudR, point[2]*cloudR])
        pygame.draw.circle(screen, vectorColors[3], [pt[0] * scale + w/2, pt[1] * scale + h/2], 3, 1)
        ind += 1
        renderPointsIndex += 1
    
    if(len(trail) > 0):
        trailind = 0
        while(trailind < (len(trail) - 1)):
            point1 = rotateZ(trail[trailind], angle)
            pt1 = pointToProjected([point1[0]*cloudR, point1[1]*cloudR, point1[2]*cloudR])
            point2 = rotateZ(trail[trailind + 1], angle)
            pt2 = pointToProjected([point2[0]*cloudR, point2[1]*cloudR, point2[2]*cloudR])
            pygame.draw.line(screen, vectorColors[4], [pt1[0] * scale + w/2, pt1[1] * scale + h/2], [pt2[0] * scale + w/2, pt2[1] * scale + h/2], 1)
            trailind += 1
        
        endd = rotateZ(trail[-1], angle)
        endpt = pointToProjected([endd[0]*cloudR, endd[1]*cloudR, endd[2]*cloudR])
        pygame.draw.line(screen, vectorColors[4], [w/2,h/2], [endpt[0] * scale + w/2, endpt[1] * scale + h/2], 1)
    
    
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
        
        render([x,y], None, [vec3D(instaccel, disp), vec3D(acce, disp, 1, True)], renderings, testPts, fmodl([x_prev, y_prev], 2 * math.pi))
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