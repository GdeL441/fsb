import time
from machine import Pin
import neopixel

class Statusled:
    def __init__(self, pin, brightness=0.6):
        self.NUM_PIXELS = 35
        self.brightness = brightness
        self.pixels = neopixel.NeoPixel(Pin(pin), self.NUM_PIXELS)
        self.set_brightness(brightness)
        
    def set_brightness(self, brightness):
        self.brightness = brightness
        
    def turn_off(self):
        self.pixels.fill((0, 0, 0))
        self.pixels.write()

    def next_object(self):
        intensity = smooth_sine(0.5)  # 0.5 Hz
        # Create breathing white-green effect
        for i in range(self.NUM_PIXELS):
            self.pixels[i] = (
                int(255 * intensity * self.brightness),  # R
                int(255 * self.brightness),  # G
                int(255 * intensity * self.brightness),  # B
            )
        self.pixels.write()

    def collection(self):
        intensity = smooth_sine(1)  # 1 Hz
        # Orange/yellow pulsing
        color = (
            int(255 * intensity * self.brightness),  # R
            int(128 * intensity * self.brightness),  # G
            0  # B
        )
        self.pixels.fill(color)
        self.pixels.write()

    def reverse(self):
        intensity = smooth_sine(1)  # 1 Hz
        # Red pulsing
        color = (
            int(255 * intensity * self.brightness),  # R
            0,  # G
            0   # B
        )
        self.pixels.fill(color)
        self.pixels.write()

    def return_home(self):
        intensity = smooth_sine(1)  # 1 Hz
        # Blue pulsing
        color = (
            0,  # R
            0,  # G
            int(255 * intensity * self.brightness)  # B
        )
        self.pixels.fill(color)
        self.pixels.write()

    def loading_animation(self):
        # Number of lit pixels in the spinning segment
        segment_length = 5
        
        # Get current position based on time
        t = time.ticks_ms() / 1000
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
                int(255 * brightness * self.brightness),  # R
                int(255 * brightness * self.brightness),  # G
                int(255 * brightness * self.brightness),  # B
            )
    
        # Show the updated pixels
        self.pixels.write()

    def connected(self, duration=1.0):
        start_time = time.ticks_ms()
        elapsed = 0        
        # Run until the animation is complete
        while elapsed < duration * 1000:
            # Calculate current progress (0.0 to 1.0)
            elapsed = time.ticks_diff(time.ticks_ms(), start_time)
            progress = min(elapsed / (duration * 1000), 1.0)
            
            # Calculate how many LEDs to light up
            lit_pixels = int(self.NUM_PIXELS * progress)
            
            # Turn off all pixels first
            self.pixels.fill((0, 0, 0))
            
            # Light up the pixels with green color
            for i in range(lit_pixels):
                # Optional: add slight brightness variation
                brightness = 0.8 + (0.2 * (i / self.NUM_PIXELS))
                
                self.pixels[i] = (
                    0,                                          # R
                    int(255 * brightness * self.brightness),    # G
                    0                                           # B
                )
            
            # Show the updated pixels
            self.pixels.write()
            
            # Small delay to control update rate
            time.sleep(0.01)
        
        # Ensure all pixels are lit at the end
        self.pixels.fill((0, int(255 * self.brightness), 0))
        self.pixels.write()
        
        # Optional: flash the completed circle a couple times to indicate completion
        for _ in range(2):
            time.sleep(0.1)
            self.pixels.fill((0, 0, 0))
            self.pixels.write()
            time.sleep(0.1)
            self.pixels.fill((0, int(255 * self.brightness), 0))
            self.pixels.write()

    def manual_control(self):
        # Number of lit pixels in the spinning segment
        segment_length = 5
        
        # Get current position based on time
        t = time.ticks_ms() / 1000
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
        
            # Set pixel color (red with gradient)
            self.pixels[pixel_pos] = (
                int(255 * brightness * self.brightness),  # R
                0,  # G
                0   # B
            )
    
        # Show the updated pixels
        self.pixels.write()


    def finished(self):
        # Create a rotating rainbow effect
        t = time.ticks_ms() / 1000
        # Speed of rotation (complete rotation every 2 seconds)
        rotation_speed = 0.5
        
        # Calculate the base hue offset based on time
        hue_offset = (t * rotation_speed) % 1.0
        
        # Turn off all pixels first
        self.pixels.fill((0, 0, 0))
        
        # Light up all pixels with a rainbow pattern
        for i in range(self.NUM_PIXELS):
            # Calculate hue for this pixel (0.0 to 1.0)
            # Divide by NUM_PIXELS to spread colors across the strip
            # Add hue_offset for rotation effect
            hue = ((i / self.NUM_PIXELS) + hue_offset) % 1.0
            
            # Convert HSV to RGB (simplified version)
            r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
            
            # Set pixel color
            self.pixels[i] = (
                int(r * 255 * self.brightness),  # R
                int(g * 255 * self.brightness),  # G
                int(b * 255 * self.brightness)   # B
            )
        
        # Show the updated pixels
        self.pixels.write()

    def collision(self):
        for _ in range(2):
            time.sleep(0.1)
            self.pixels.fill((0, 0, 0))
            self.pixels.write()
            time.sleep(0.1)
            self.pixels.fill((int(255 * self.brightness), 0, 0))
            self.pixels.write()

    def calibration(self, duration = 0.66):
        start_time = time.ticks_ms()
        elapsed = 0        
        # Run until the animation is complete
        while elapsed < duration * 1000:
            # Calculate current progress (0.0 to 1.0)
            elapsed = time.ticks_diff(time.ticks_ms(), start_time)
            progress = min(elapsed / (duration * 1000), 1.0)
            
            # Calculate how many LEDs to light up
            lit_pixels = int(self.NUM_PIXELS * progress)
            
            # Turn off all pixels first
            self.pixels.fill((0, 0, 0))
            
            # Light up the pixels with color
            for i in range(lit_pixels):
                # Optional: add slight brightness variation
                brightness = 0.8 + (0.2 * (i / self.NUM_PIXELS))
                
                self.pixels[i] = (
                    0,                                          # R
                    int(255 * brightness * self.brightness),    # G
                    int(255 * brightness * self.brightness)     # B
                )
            
            # Show the updated pixels
            self.pixels.write()
            
            # Small delay to control update rate
            time.sleep(0.01)
        
        # Ensure all pixels are lit at the end
        self.pixels.fill((0, int(255 * self.brightness), int(255 * self.brightness)))
        self.pixels.write()
    
    def calibration_finished(self):
        for _ in range(2):
            time.sleep(0.1)
            self.pixels.fill((0, 0, 0))
            self.pixels.write()
            time.sleep(0.1)
            self.pixels.fill((0, int(255 * self.brightness), int(255 * self.brightness)))
            self.pixels.write()


    def waiting_for_orders(self, duration = 1.0):
        # Number of lit pixels in the spinning segment
        segment_length = 5
        
        # Get current position based on time
        t = time.ticks_ms() / 1000
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
                int(255 * brightness * self.brightness),  # B
            )
    
        # Show the updated pixels
        self.pixels.write()


def smooth_sine(frequency):
    """
    Creates a smooth sine wave with given frequency (Hz)
    Returns value between 0 and 1
    """
    from math import sin, pi
    t = time.ticks_ms() / 1000  # Convert to seconds
    return (sin(2 * pi * frequency * t) + 1) / 2

# Helper function to convert HSV to RGB
def hsv_to_rgb(h, s, v):
    """
    Convert HSV color to RGB
    h: hue (0.0 to 1.0)
    s: saturation (0.0 to 1.0)
    v: value (0.0 to 1.0)
    Returns: (r, g, b) tuple with values 0.0 to 1.0
    """
    if s == 0.0:
        return (v, v, v)
    
    i = int(h * 6)
    f = (h * 6) - i
    p = v * (1 - s)
    q = v * (1 - s * f)
    t = v * (1 - s * (1 - f))
    
    i = i % 6
    if i == 0:
        return (v, t, p)
    elif i == 1:
        return (q, v, p)
    elif i == 2:
        return (p, v, t)
    elif i == 3:
        return (p, q, v)
    elif i == 4:
        return (t, p, v)
    else:
        return (v, p, q)
