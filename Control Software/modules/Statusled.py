import pwmio # type: ignore
import digitalio # type: ignore
from math import sin, pi
import time

class Statusled:
    def __init__(self, red_pin, green_pin, blue_pin, white_pin):
        self.red = pwmio.PWMOut(red_pin, frequency=1000)
        self.green = pwmio.PWMOut(green_pin, frequency=1000)
        self.blue = pwmio.PWMOut(blue_pin, frequency=1000)
        self.white = pwmio.PWMOut(white_pin, frequency=1000)

    def turn_off(self):
        self.red.duty_cycle = 0
        self.green.duty_cycle = 0
        self.blue.duty_cycle = 0
        self.white.duty_cycle = 0

    def next_object(self):
        intensity = white_green_sine()
        self.red.duty_cycle = intensity_to_duty_cycle(intensity)
        self.green.duty_cycle = intensity_to_duty_cycle(1)
        self.blue.duty_cycle = intensity_to_duty_cycle(intensity)
        self.white.duty_cycle = intensity_to_duty_cycle(intensity)

    def collection(self):
        intensity = cyclic()
        self.red.duty_cycle = intensity_to_duty_cycle(intensity)
        self.green.duty_cycle = intensity_to_duty_cycle(intensity / 2)
        self.blue.duty_cycle = intensity_to_duty_cycle(0)
        self.white.duty_cycle = intensity_to_duty_cycle(0)

    def reverse(self):
        intensity = cyclic()
        self.red.duty_cycle = intensity_to_duty_cycle(intensity)
        self.green.duty_cycle = intensity_to_duty_cycle(0)
        self.blue.duty_cycle = intensity_to_duty_cycle(0)
        self.white.duty_cycle = intensity_to_duty_cycle(0)

    def return_home(self):
        intensity = cyclic()
        self.red.duty_cycle = intensity_to_duty_cycle(0)
        self.green.duty_cycle = intensity_to_duty_cycle(0)
        self.blue.duty_cycle = intensity_to_duty_cycle(intensity)
        self.white.duty_cycle = intensity_to_duty_cycle(0)

def white_green_sine():
    t = time.monotonic()
    return 0.5 + 0.5 * sin(2*pi*t)

def cyclic():
    t = int(time.monotonic())
    if t % 2 == 0:
        return 1
    else:
        return 0
    
def intensity_to_duty_cycle(intensity):
    duty_cycle = int(intensity * 65535)
    return duty_cycle
