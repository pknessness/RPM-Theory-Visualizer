import random

x = 0
y = 0

def execute(desired_g: float):
    global x, y
    x += (random.random() - 0.5) * 0.2
    y += (random.random() - 0.5) * 0.2
    #return [x, y]
    return [10,10]