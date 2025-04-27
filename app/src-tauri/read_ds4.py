import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import pygame
import time
import sys
pygame.init()
pygame.joystick.init()


controller = pygame.joystick.Joystick(0)
controller.init()

axis = {}
button = {}

# these are the identifiers for the PS4's accelerometers
AXIS_X = 3
AXIS_Y = 4
AXIS_Z = 1
AXIS_W = 0

# variables we'll store the rotations in, initialised to zero
rot_x = 0.0
rot_y = 0.0
rot_z = 0.0
rot_w = 0.0

# main loop
while True:

    # copy rot_x/rot_y into axis[] in case we don't read any
    axis[AXIS_X] = rot_x
    axis[AXIS_Y] = rot_y
    axis[AXIS_Z] = rot_z
    axis[AXIS_W] = rot_w

    # retrieve any events ...
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            axis[event.axis] = round(event.value,2)
        elif event.type == pygame.JOYBUTTONDOWN:
            button[event.button] = True
        elif event.type == pygame.JOYBUTTONUP:
            button[event.button] = False

    rot_x = axis[AXIS_X]
    rot_y = axis[AXIS_Y] 
    rot_z = axis[AXIS_Z]
    rot_w = axis[AXIS_W] 
    print(f"{axis[AXIS_Z]},{axis[AXIS_W]},{axis[AXIS_X]},{axis[AXIS_Y]}\n", end="")
    sys.stdout.flush()
    pygame.time.wait(30)
