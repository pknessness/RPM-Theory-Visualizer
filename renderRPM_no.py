import numpy as np
import sys, math

import pygame
from pygame.locals import *

pygame.init()

# camLocation = np.array([50,50,50])
# camAspect = 
# camDepth = 100

# camDirection = -camLocation

# print(camDirection)

# width, height = 640, 480

# fNear = 0.1
# fFar = 1000
# fFov = 90
# fAspect = height/width
# fFovRad = 1/ math.tan(fFov * 0.5 / 180 * math.pi)

# projection_matrix = [
#     [fAspect * fFovRad, 0,                  0,                  0],
#     [0,                 fFovRad,            0,                  0],
#     [0,                 0,       fFar / (fFar - fNear), (-fFar * fNear) / (fFar - fNear)],
#     [0,                 0,                  1.0,                  0],
# ]

# fps = 60
# fpsClock = pygame.time.Clock()


# screen = pygame.display.set_mode((width, height))

# quads = [] #[[x,y,z],...]

# # Game loop.
# while True:
#     screen.fill((0, 0, 0))
    
#     for event in pygame.event.get():
#         if event.type == QUIT:
#             pygame.quit()
#             sys.exit()
    
#     # Update.
    
#     # Draw.
    
#     pygame.display.flip()
#     fpsClock.tick(fps)

'''
Basic 3D rendering example using pygame library in Python
Author: Goran Trlin
Find more tutorials and code samples on:
https://playandlearntocode.com
'''
import math
import pygame
import numpy as np

# window settings:
screen_width, screen_height = 800, 600

# init pygame settings:
pygame.init()
pygame.display.set_caption('Python 3D rendering example')
screen = pygame.display.set_mode((screen_width, screen_height))
clock = pygame.time.Clock()
cube_center = {'y': screen_height // 2, 'x': screen_width // 2}
fps = 60

# colors:
background_color = (240, 235, 240)
cube_color = (255, 0, 70)
circle_color = (255, 0, 70)

# 3D points:
cube_vertices = [
    [-1, -1, 1],
    [1, -1, 1],
    [1, 1, 1],
    [-1, 1, 1],
    [-1, -1, -1],
    [1, -1, -1],
    [1, 1, -1],
    [-1, 1, -1]
]

# cube sides, each entry is an index in cube_vertices:
cube_sides = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [1, 2, 6, 5],
    [0, 3, 7, 4],
    [0, 1, 5, 4],
    [2, 3, 7, 6]
]

angleX = 0
angleY = 0
angleZ = 0
angular_change = 0.02
distance = 2
scale = 200
circle_radius = 8
running = True

# main game loop:
while running:
    clock.tick(fps)
    screen.fill(background_color)
    angleX += angular_change
    angleY += angular_change
    #angleZ += angular_change

    # process window events:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # prepare rotation matrix:
    rotation_matrix_x = [
        [1, 0, 0],
        [0, math.cos(angleX), -math.sin(angleX)],
        [0, math.sin(angleX), math.cos(angleX)]
    ]

    rotation_matrix_y = [
        [math.cos(angleY), 0, -math.sin(angleY)],
        [0, 1, 0],
        [math.sin(angleY), 0, math.cos(angleY)]
    ]

    rotation_matrix_z = [
        [math.cos(angleZ), -math.sin(angleZ), 0],
        [math.sin(angleZ), math.cos(angleZ), 0],
        [0, 0, 1]
    ]

    z = 1 / distance

    projection_matrix = [
        [z, 0, 0],
        [0, z, 0]
    ]

    # compute the final matrix:
    rotation_matrix = np.matmul(rotation_matrix_x, rotation_matrix_y)
    rotation_matrix = np.matmul(rotation_matrix_z, rotation_matrix)
    projection_rotation_matrix = np.matmul(projection_matrix, rotation_matrix)
    projected_vertices = []

    # project vertices to 2D coordinates:
    for p in cube_vertices:
        # apply projection matrix to each cube vertex:
        projected = np.matmul(projection_rotation_matrix, np.transpose(p))
        x = cube_center['x'] + projected[0] * scale
        y = cube_center['y'] + projected[1] * scale
        projected_vertices.append([x, y])

    # draw vertices as circles:
    for p in projected_vertices:
        pygame.draw.circle(screen, circle_color, (p[0], p[1]), circle_radius)

    # draw cube sides:
    counter = 1
    for side in cube_sides:
        side_coords = [
            (projected_vertices[side[0]][0], projected_vertices[side[0]][1]),
            (projected_vertices[side[1]][0], projected_vertices[side[1]][1]),
            (projected_vertices[side[2]][0], projected_vertices[side[2]][1]),
            (projected_vertices[side[3]][0], projected_vertices[side[3]][1])
        ]
        pygame.draw.polygon(screen, (
            int(cube_color[0] * counter * 0.05), int(cube_color[1] * counter * 0.05),
            int(cube_color[2] * counter * 0.05)), side_coords)
        counter += 1

    # present the new image to screen:
    pygame.display.update()

pygame.quit()