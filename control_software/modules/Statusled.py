import neopixel # type: ignore
import board # type: ignore
from math import sin, pi
import time

class Statusled:
    def __init__(self, pin):
        self.NUM_PIXELS = 35
        self.pixels = neopixel.NeoPixel(pin, self.NUM_PIXELS, brightness=0.3, auto_write=False)
        
    def turn_off(self):
        self.pixels.fill((0, 0, 0))
        self.pixels.show()

    def next_object(self):
        intensity = white_green_sine()
        # Create breathing white-green effect
        for i in range(self.NUM_PIXELS):
            self.pixels[i] = (
                int(255 * intensity),  # R
                255,                   # G
                int(255 * intensity)   # B
            )
        self.pixels.show()

    def collection(self):
        intensity = cyclic()
        # Orange/yellow pulsing
        color = (
            int(255 * intensity),      # R
            int(128 * intensity),      # G
            0                          # B
        )
        self.pixels.fill(color)
        self.pixels.show()

    def reverse(self):
        intensity = cyclic()
        # Red pulsing
        color = (
            int(255 * intensity),  # R
            0,                     # G
            0                      # B
        )
        self.pixels.fill(color)
        self.pixels.show()

    def return_home(self):
        intensity = cyclic()
        # Blue pulsing
        color = (
            0,                     # R
            0,                     # G
            int(255 * intensity)   # B
        )
        self.pixels.fill(color)
        self.pixels.show()

def white_green_sine():
    t = time.monotonic()
    return 0.5 + 0.5 * sin(2*pi*t)

def cyclic():
    t = int(time.monotonic())
    if t % 2 == 0:
        return 1
    else:
        return 0
