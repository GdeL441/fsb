import neopixel # type: ignore
import board # type: ignore
from math import sin, pi
import time

class Statusled:
    def __init__(self, pin, brightness=0.3):
        self.NUM_PIXELS = 35
        brightness = self.brightness
        self.pixels = neopixel.NeoPixel(pin, self.NUM_PIXELS, brightness=brightness, auto_write=False)
        
    def turn_off(self):
        self.pixels.fill((0, 0, 0))
        self.pixels.show()

    def next_object(self):
        intensity = smooth_sine(0.5)  # 0.5 Hz - adjust this value to change animation speed
        # Create breathing white-green effect
        for i in range(self.NUM_PIXELS):
            self.pixels[i] = (
                int(255 * intensity),  # R
                255,                   # G
                int(255 * intensity)   # B
            )
        self.pixels.show()

    def collection(self):
        intensity = smooth_sine(1)  # 1 Hz
        # Orange/yellow pulsing
        color = (
            int(255 * intensity),      # R
            int(128 * intensity),      # G
            0                          # B
        )
        self.pixels.fill(color)
        self.pixels.show()

    def reverse(self):
        intensity = smooth_sine(1)  # 1 Hz
        # Red pulsing
        color = (
            int(255 * intensity),  # R
            0,                     # G
            0                      # B
        )
        self.pixels.fill(color)
        self.pixels.show()

    def return_home(self):
        intensity = smooth_sine(1)  # 1 Hz
        # Blue pulsing
        color = (
            0,                     # R
            0,                     # G
            int(255 * intensity)   # B
        )
        self.pixels.fill(color)
        self.pixels.show()

def smooth_sine(frequency):
    """
    Creates a smooth sine wave with given frequency (Hz)
    Returns value between 0 and 1
    """
    t = time.monotonic()
    return (sin(2 * pi * frequency * t) + 1) / 2
