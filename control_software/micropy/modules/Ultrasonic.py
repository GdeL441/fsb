from machine import Pin, time_pulse_us
import utime

# From https://github.com/kfricke/micropython-us100

class Ultrasonic:
    """Read the US-100 sonar sensor in UART mode."""

    def __init__(self, uart, threshold_cm):
        self.uart = uart
        uart.init(baudrate=9600, bits=8, parity=None, stop=1)
        utime.sleep_ms(100)
        self.buf = bytearray(2)
        self.t = 0
        self.threshold = threshold_cm

    def distance(self):
        """Read the temperature compensated distance im millimeters."""
        self.buf[0] = 0
        self.buf[1] = 0
        self.uart.write(b'\x55')
        self.t = utime.ticks_ms()
        while not self.uart.any():
            if utime.ticks_diff(utime.ticks_ms(), self.t) > 5: # Changed to 10ms, while the car is normally driving, it would always
                                                                # timeout (±200ms), this makes the main loop run slower
                print('Timeout (or other error) while reading from US100 sensor!')
        self.uart.readinto(self.buf, 2)

        distance_mm = (self.buf[0] * 256) + self.buf[1]
        distance_cm = distance_mm / 10
        return int(distance_cm)
    
    def detect(self):
        """Read the temperature compensated distance im millimeters."""
        self.buf[0] = 0
        self.buf[1] = 0
        self.uart.write(b'\x55')
        self.t = utime.ticks_ms()
        while not self.uart.any():
            if utime.ticks_diff(utime.ticks_ms(), self.t) > 5: # Changed to 10ms, while the car is normally driving, it would always
                                                                # timeout (±200ms), this makes the main loop run slower
                return False # In 10ms sound travels 343cm, more than enough for accurate collision detection. 
        self.uart.readinto(self.buf, 2)

        distance_mm = (self.buf[0] * 256) + self.buf[1]
        distance_cm = distance_mm / 10
        return distance_cm < self.threshold  
    
    def set_threshold(self, new_threshold):
        self.threshold = new_threshold
