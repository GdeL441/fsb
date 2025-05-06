import neopixel  # type: ignore
from math import sin, pi
import time


class Statusled:
    def __init__(self, pin, brightness=0.3):
        self.NUM_PIXELS = 35
        self.brightness = brightness
        self.pixels = neopixel.NeoPixel(pin, self.NUM_PIXELS, brightness=brightness, auto_write=False)

    def turn_off(self):
        self.pixels.fill((0, 0, 0))
        self.pixels.show()

    def next_object(self):
        intensity = smooth_sine(
            0.5
        )  # 0.5 Hz - adjust this value to change animation speed
        # Create breathing white-green effect
        for i in range(self.NUM_PIXELS):
            self.pixels[i] = (
                int(255 * intensity),  # R
                255,  # G
                int(255 * intensity),  # B
            )
        self.pixels.show()

    def collection(self):
        intensity = smooth_sine(1)  # 1 Hz
        # Orange/yellow pulsing
        color = (int(255 * intensity), int(128 * intensity), 0)  # R  # G  # B
        self.pixels.fill(color)
        self.pixels.show()

    def reverse(self):
        intensity = smooth_sine(1)  # 1 Hz
        # Red pulsing
        color = (int(255 * intensity), 0, 0)  # R  # G  # B
        self.pixels.fill(color)
        self.pixels.show()

    def return_home(self):
        intensity = smooth_sine(1)  # 1 Hz
        # Blue pulsing
        color = (0, 0, int(255 * intensity))  # R  # G  # B
        self.pixels.fill(color)
        self.pixels.show()

    def loading_animation(self):
        # Number of lit pixels in the spinning segment
        segment_length = 5
        
        # Get current position based on time
        t = time.monotonic()
        # Speed of rotation (complete rotation every 1.5 seconds)
        rotation_speed = 0.67  
    
        # Calculate the position of the first pixel in the segment
        position = int((t * rotation_speed * self.NUM_PIXELS) % self.NUM_PIXELS)
        
        # Turn off all pixels first
        self.pixels.fill((0, 0, 0))
        
        # Light up the segment with a gradient effect
        for i in range(segment_length):
            # Calculate pixel position with wraparound
            pixel_pos = (position + i) % self.NUM_PIXELS
        
            # Create a gradient effect within the segment (brighter at the front)
            brightness = 1.0 - (i / segment_length)
        
            # Set pixel color (white with gradient)
            self.pixels[pixel_pos] = (
                int(255 * brightness),  # R
                int(255 * brightness),  # G
                int(255 * brightness),  # B
            )
    
        # Show the updated pixels
        self.pixels.show()

    def connected(self, duration=1.0):
        start_time = time.monotonic()
        elapsed = 0        
        # Run until the animation is complete
        while elapsed < duration:
            # Calculate current progress (0.0 to 1.0)
            elapsed = time.monotonic() - start_time
            progress = min(elapsed / duration, 1.0)
            
            # Calculate how many LEDs to light up
            lit_pixels = int(self.NUM_PIXELS * progress)
            
            # Turn off all pixels first
            self.pixels.fill((0, 0, 0))
            
            # Light up the pixels with green color
            for i in range(lit_pixels):
                # Optional: add slight brightness variation
                brightness = 0.8 + (0.2 * (i / self.NUM_PIXELS))
                
                self.pixels[i] = (
                    0,                      # R
                    int(255 * brightness),  # G
                    0                       # B
                )
            
            # Show the updated pixels
            self.pixels.show()
            
            # Small delay to control update rate
            time.sleep(0.01)
        
        # Ensure all pixels are lit at the end
        self.pixels.fill((0, 255, 0))
        self.pixels.show()
        
        # Optional: flash the completed circle a couple times to indicate completion
        for _ in range(2):
            time.sleep(0.1)
            self.pixels.fill((0, 0, 0))
            self.pixels.show()
            time.sleep(0.1)
            self.pixels.fill((0, 255, 0))
            self.pixels.show()

    def manual_control(self):
        # Number of lit pixels in the spinning segment
        segment_length = 5
        
        # Get current position based on time
        t = time.monotonic()
        # Speed of rotation (complete rotation every 1.5 seconds)
        rotation_speed = 0.67  
    
        # Calculate the position of the first pixel in the segment
        position = int((t * rotation_speed * self.NUM_PIXELS) % self.NUM_PIXELS)
        
        # Turn off all pixels first
        self.pixels.fill((0, 0, 0))
        
        # Light up the segment with a gradient effect
        for i in range(segment_length):
            # Calculate pixel position with wraparound
            pixel_pos = (position + i) % self.NUM_PIXELS
        
            # Create a gradient effect within the segment (brighter at the front)
            brightness = 1.0 - (i / segment_length)
        
            # Set pixel color (white with gradient)
            self.pixels[pixel_pos] = (
                int(255 * brightness),  # R
                int(0 * brightness),  # G
                int(0 * brightness),  # B
            )
    
        # Show the updated pixels
        self.pixels.show()


    def waiting_for_orders(self):
        # Number of lit pixels in the spinning segment
        segment_length = 5
        
        # Get current position based on time
        t = time.monotonic()
        # Speed of rotation (complete rotation every 1.5 seconds)
        rotation_speed = 0.67  
    
        # Calculate the position of the first pixel in the segment
        position = int((t * rotation_speed * self.NUM_PIXELS) % self.NUM_PIXELS)
        
        # Turn off all pixels first
        self.pixels.fill((0, 0, 0))
        
        # Light up the segment with a gradient effect
        for i in range(segment_length):
            # Calculate pixel position with wraparound
            pixel_pos = (position + i) % self.NUM_PIXELS
        
            # Create a gradient effect within the segment (brighter at the front)
            brightness = 1.0 - (i / segment_length)
        
            # Set pixel color (white with gradient)
            self.pixels[pixel_pos] = (
                0,  # R
                0,  # G
                int(255 * brightness),  # B
            )
    
        # Show the updated pixels
        self.pixels.show()

    def collision(self):
        for _ in range(2):
            time.sleep(0.1)
            self.pixels.fill((0, 0, 0))
            self.pixels.show()
            time.sleep(0.1)
            self.pixels.fill((255, 0, 0))
            self.pixels.show()

    def calibration(self, duration = 0.66):
        start_time = time.monotonic()
        elapsed = 0        
        # Run until the animation is complete
        while elapsed < duration:
            # Calculate current progress (0.0 to 1.0)
            elapsed = time.monotonic() - start_time
            progress = min(elapsed / duration, 1.0)
            
            # Calculate how many LEDs to light up
            lit_pixels = int(self.NUM_PIXELS * progress)
            
            # Turn off all pixels first
            self.pixels.fill((0, 0, 0))
            
            # Light up the pixels with color
            for i in range(lit_pixels):
                # Optional: add slight brightness variation
                brightness = 0.8 + (0.2 * (i / self.NUM_PIXELS))
                
                self.pixels[i] = (
                    0,                      # R
                    int(255 * brightness),  # G
                    int(255 * brightness)   # B
                )
            
            # Show the updated pixels
            self.pixels.show()
            
            # Small delay to control update rate
            time.sleep(0.01)
        
        # Ensure all pixels are lit at the end
        self.pixels.fill((0, 255, 255))
        self.pixels.show()
        
        # Optional: flash the completed circle a couple times to indicate completion
        for _ in range(2):
            time.sleep(0.1)
            self.pixels.fill((0, 0, 0))
            self.pixels.show()
            time.sleep(0.1)
            self.pixels.fill((0, 255, 255))
            self.pixels.show()

def smooth_sine(frequency):
    """
    Creates a smooth sine wave with given frequency (Hz)
    Returns value between 0 and 1
    """
    t = time.monotonic()
    return (sin(2 * pi * frequency * t) + 1) / 2
