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
cameraDir = projectionPanelPos * -1 #[0,5,2]
cameraSideways = np.array([0,0,1]) #can be undefined and will autogen

vertices = []
squares = []
quads = []

backgroundColor = "#000000"
frameColor = "#FFFFFF"
vectorColorX = "#E85F2A"
vectorColorY = "#5FE82A"
vectorColorZ = "#2A5FE8"

vectorColors = ["#E85F2A", "#5FE82A", "#2A5FE8", "#9C27B0", "#F48FB1", "#FFB74D", "#00BCD4"]

class vec3D:
    def __init__(self, dir, src = [0,0,0]):
        self.src = src
        self.dir = dir

pygame.init()

pygame.font.init() # you have to call this at the start, 
basic_font = pygame.font.SysFont('Agency FB', 30)

fps = 60
fpsClock = pygame.time.Clock()

start_width, start_height = 640, 480
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
    v2 = [
        vertex[0] - projectionPanelPos[0],
        vertex[1] - projectionPanelPos[1],
        vertex[2] - projectionPanelPos[2]]
    cs = math.cos(theta)
    sn = math.sin(theta)
    v2 = [v2[0], v2[1] * cs - v2[2] * sn, v2[1] * sn + v2[2] * cs]
    return v2

def rotateTranslateY(vertex):
    theta = math.atan2(cameraDir[2],cameraDir[0])
    # print("theta",theta)
    v2 = [
        vertex[0] - projectionPanelPos[0],
        vertex[1] - projectionPanelPos[1],
        vertex[2] - projectionPanelPos[2]]
    # print("v2",v2)
    cs = math.cos(theta)
    sn = math.sin(theta)
    #v2 = [v2[0], v2[1] * cs - v2[2] * sn, v2[1] * sn + v2[2] * cs]
    v2 = [v2[0] * cs - v2[2] * sn, v2[1], v2[0] * sn + v2[2] * cs]
    # print("v2",v2)
    return v2

def rotateTranslateZ(vertex):
    theta = math.atan2(cameraDir[1],cameraDir[0])
    print("theta",theta)
    v2 = [
        vertex[0] - projectionPanelPos[0],
        vertex[1] - projectionPanelPos[1],
        vertex[2] - projectionPanelPos[2]]
    print("v2",v2)
    cs = math.cos(theta)
    sn = math.sin(theta)
    #v2 = [v2[0], v2[1] * cs - v2[2] * sn, v2[1] * sn + v2[2] * cs]
    v2 = [v2[0] * cs - v2[1] * sn, v2[0] * sn + v2[1] * cs, v2[2]]
    print("v2",v2)
    return v2

def rotateX(vertex, angle):
    cs = math.cos(angle)
    sn = math.sin(angle)
    v2 = [vertex[0], vertex[1] * cs - vertex[2] * sn, vertex[1] * sn + vertex[2] * cs]
    return v2

def rotateY(vertex, angle):
    cs = math.cos(angle)
    sn = math.sin(angle)
    v2 = [vertex[0] * cs - vertex[2] * sn, vertex[1],  vertex[0] * sn + vertex[2] * cs]
    return v2

def rotateZ(vertex, angle):
    cs = math.cos(angle)
    sn = math.sin(angle)
    v2 = [vertex[0] * cs - vertex[1] * sn, vertex[0] * sn + vertex[1] * cs, vertex[2]]
    return v2

def fillQuad(quad: Quad, scale: float, pixelOffset, color = "#FFFFFF"):
    # ctx.beginPath()
    # ctx.fillStyle = color
    # ctx.moveTo(quad.a[0] * scale + pixelOffset[0], quad.a[1] * scale + pixelOffset[1])
    # ctx.lineTo(quad.b[0] * scale + pixelOffset[0], quad.b[1] * scale + pixelOffset[1])
    # ctx.lineTo(quad.c[0] * scale + pixelOffset[0], quad.c[1] * scale + pixelOffset[1])
    # ctx.lineTo(quad.d[0] * scale + pixelOffset[0], quad.d[1] * scale + pixelOffset[1])
    # ctx.lineTo(quad.a[0] * scale + pixelOffset[0], quad.a[1] * scale + pixelOffset[1])
    # ctx.fillStyle = "#FFFFFF"
    # ctx.fill()
    
    pygame.draw.line(screen, pygame.Color(color), (quad.a[0] * scale + pixelOffset[0], quad.a[1] * scale + pixelOffset[1]), (quad.b[0] * scale + pixelOffset[0], quad.b[1] * scale + pixelOffset[1]))
    pygame.draw.line(screen, pygame.Color(color), (quad.b[0] * scale + pixelOffset[0], quad.b[1] * scale + pixelOffset[1]), (quad.c[0] * scale + pixelOffset[0], quad.c[1] * scale + pixelOffset[1]))
    pygame.draw.line(screen, pygame.Color(color), (quad.c[0] * scale + pixelOffset[0], quad.c[1] * scale + pixelOffset[1]), (quad.d[0] * scale + pixelOffset[0], quad.d[1] * scale + pixelOffset[1]))
    pygame.draw.line(screen, pygame.Color(color), (quad.d[0] * scale + pixelOffset[0], quad.d[1] * scale + pixelOffset[1]), (quad.a[0] * scale + pixelOffset[0], quad.a[1] * scale + pixelOffset[1]))


def strokeQuad(quad: Quad, scale: float, pixelOffset, color = "#FFFFFF", lineWidth = 2):
    # ctx.beginPath()
    # ctx.strokeStyle = color
    # ctx.lineWidth = lineWidth
    # ctx.moveTo(quad.a[0] * scale + pixelOffset[0], quad.a[1] * scale + pixelOffset[1])
    # ctx.lineTo(quad.b[0] * scale + pixelOffset[0], quad.b[1] * scale + pixelOffset[1])
    # ctx.lineTo(quad.c[0] * scale + pixelOffset[0], quad.c[1] * scale + pixelOffset[1])
    # ctx.lineTo(quad.d[0] * scale + pixelOffset[0], quad.d[1] * scale + pixelOffset[1])
    # ctx.lineTo(quad.a[0] * scale + pixelOffset[0], quad.a[1] * scale + pixelOffset[1])
    # # ctx.fillStyle = "#FFFFFF"
    # ctx.stroke()
    
    pygame.draw.line(screen, pygame.Color(color), (quad.a[0] * scale + pixelOffset[0], quad.a[1] * scale + pixelOffset[1]), (quad.b[0] * scale + pixelOffset[0], quad.b[1] * scale + pixelOffset[1]), lineWidth)
    pygame.draw.line(screen, pygame.Color(color), (quad.b[0] * scale + pixelOffset[0], quad.b[1] * scale + pixelOffset[1]), (quad.c[0] * scale + pixelOffset[0], quad.c[1] * scale + pixelOffset[1]), lineWidth)
    pygame.draw.line(screen, pygame.Color(color), (quad.c[0] * scale + pixelOffset[0], quad.c[1] * scale + pixelOffset[1]), (quad.d[0] * scale + pixelOffset[0], quad.d[1] * scale + pixelOffset[1]), lineWidth)
    pygame.draw.line(screen, pygame.Color(color), (quad.d[0] * scale + pixelOffset[0], quad.d[1] * scale + pixelOffset[1]), (quad.a[0] * scale + pixelOffset[0], quad.a[1] * scale + pixelOffset[1]), lineWidth)



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
    # # vert = vertices.length
    # square = [-1,-1,-1,-1]
    # # squares.push([-1,-1,-1,-1])
    # vrt = search(vertices,e)
    # # console.log(vrt)
    # if(vrt == -1){
    #     square[0] = vertices.length
    #     vertices.push(e)
    # }else{
    #     square[0] = vrt
    # }
    # vrt = search(vertices,f)
    # if(vrt == -1){
    #     square[1] = vertices.length
    #     vertices.push(f)
        
    # }else{
    #     square[1] = vrt
    # }
    # vrt = search(vertices,g)
    # if(vrt == -1){
    #     square[2] = vertices.length
    #     vertices.push(g)
        
    # }else{
    #     square[2] = vrt
    # }
    # vrt = search(vertices,h)
    # if(vrt == -1){
    #     square[3] = vertices.length
    #     vertices.push(h)
        
    # }else{
    #     square[3] = vrt
    # }
    
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

def drawing(color: str = "#FFFFFF", reset = True):
    global vertices, squares, quads
    for square in squares:
        q = squareToQuad(square)
        # console.log(squares[i])
        # console.log(q)
        # fillQuad(q,height/projectionPanelHeight,[width/2,height/2])
        w, h = pygame.display.get_window_size()
        strokeQuad(q,h/projectionPanelHeight,[w/2,h/2], color, 1)
        # print(q,height/projectionPanelHeight,[width/2,height/2])
    if(reset):
        vertices = []
        squares = []
        quads = []
    
def drawVectors(vectors: list):
    for vector in vectors:
        q = 0
        # console.log(squares[i])
        # console.log(q)
        # fillQuad(q,height/projectionPanelHeight,[width/2,height/2])
        w, h = pygame.display.get_window_size()
        strokeQuad(q,h/projectionPanelHeight,[w/2,h/2], "#FFFFFF", 1)
        # print(q,height/projectionPanelHeight,[width/2,height/2])

# def stuff():
#     addSquare([-1,0,-1],[1,0,-1],[1,0,1],[-1,0,1])
#     drawing()
#     print(vertices)
#     print(squares)
#     print(quads)

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

def render(actual: list, desired: list, vectors: list[vec3D], text: list):
    global x, cameraDir, projectionPanelPos, angle
    global vertices, squares, quads
    
    screen.fill(backgroundColor)
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    
    move_speed = 1
    rotate_speed = 0.01
    
    keys=pygame.key.get_pressed()
    if keys[K_w]:
        projectionPanelPos += [0,move_speed,0]
        cameraDir = projectionPanelPos * -1
    if keys[K_s]:
        projectionPanelPos += [0,-move_speed,0]
        cameraDir = projectionPanelPos * -1
    if keys[K_a]:
        angle += rotate_speed
    if keys[K_d]:
        angle -= rotate_speed
    
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
        createRect([v.dir[0]/2 + v.src[0],v.dir[2]/2 + v.src[2],v.dir[1]/2 + v.src[1]], v.dir[0], v.dir[2], v.dir[1])
        
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], actual[1])
        for i in range(len(vertices)):
            vertices[i] = rotateX(vertices[i], actual[0])
        for i in range(len(vertices)):
            vertices[i] = rotateZ(vertices[i], angle)
        
        drawing(vectorColors[(cnt)%len(vectorColors)])
        cnt += 1
        
        # createRect([v[0]/2,0,0], v[0], 0, 0)
        
        # for i in range(len(vertices)):
        #     vertices[i] = rotateZ(vertices[i], actual[1])
        # for i in range(len(vertices)):
        #     vertices[i] = rotateX(vertices[i], actual[0])
        # for i in range(len(vertices)):
        #     vertices[i] = rotateZ(vertices[i], angle)
        
        # drawing(vectorColorX)
        
        # createRect([0,0,v[1]/2], 0, 0, v[1])
        
        # for i in range(len(vertices)):
        #     vertices[i] = rotateZ(vertices[i], actual[1])
        # for i in range(len(vertices)):
        #     vertices[i] = rotateX(vertices[i], actual[0])
        # for i in range(len(vertices)):
        #     vertices[i] = rotateZ(vertices[i], angle)
        
        # drawing(vectorColorY)
        
        # createRect([0,v[2]/2,0], 0, v[2], 0)
        
        # for i in range(len(vertices)):
        #     vertices[i] = rotateZ(vertices[i], actual[1])
        # for i in range(len(vertices)):
        #     vertices[i] = rotateX(vertices[i], actual[0])
        # for i in range(len(vertices)):
        #     vertices[i] = rotateZ(vertices[i], angle)
        
        # drawing(vectorColorZ)
        
        # addSquare(
        #     [center[0] + x/2,center[1] + y/2,center[2] + z/2],
        #     [center[0] + x/2,center[1] + y/2,center[2] - z/2],
        #     [center[0] + x/2,center[1] - y/2,center[2] - z/2],
        #     [center[0] + x/2,center[1] - y/2,center[2] + z/2])
    
    drawing()
    
    writeHeight = 0
    for line in text:
        text_surface = basic_font.render(line.text, False, line.color)
        screen.blit(text_surface, (0,writeHeight))
        writeHeight += 40
    
    pygame.display.flip()
    fpsClock.tick(fps)
        
x = 0
y = 0

MANUAL_POS_SCALING = 0.01

if __name__ == "__main__":
    instaccel = [0.0,0.0,0.0]
    
    prevMouse = [False, False, False]
    dragPos = [[0,0],[0,0],[0,0]]
    
    while(1):
        mouse = pygame.mouse.get_pressed(num_buttons=3)
        
        renderings = [
            WriteLine(f"OUTER ANGLE: {x:.2f}/{fmod(x,2*math.pi):.2f} rad", (200,55,25)),
            WriteLine(f"INNER ANGLE: {y:.2f}/{fmod(y,2*math.pi):.2f} rad", (55,200,25))]
        
        renderings.append(WriteLine(f"INST: X:{instaccel[0]:.2f} Y:{instaccel[1]:.2f} Z:{instaccel[2]:.2f}", (205,100,200)))
        
        disp = [0,10,0]
        
        render([x,y], None, [vec3D([instaccel[0],0,0], disp), vec3D([0,instaccel[1],0], disp), vec3D([0,0, instaccel[2]], disp)], renderings)
        # x += 0.013291
        # y += 0.023129
        
        mouse_pos = pygame.mouse.get_pos()
        if(mouse[0] and prevMouse[0]):
            disp = [mouse_pos[0] - dragPos[0][0], mouse_pos[1] - dragPos[0][1]]
            x += disp[1] * MANUAL_POS_SCALING
            y += disp[0] * MANUAL_POS_SCALING
        if(mouse[0]):
            dragPos[0] = mouse_pos
        
        instaccel = [0,-9.81, 0]
        
        instaccel = rotateX(instaccel, x)
        instaccel = rotateY(instaccel, -y)
        
        #instaccel = rotateZ(instaccel, -y)
        prevMouse = mouse
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