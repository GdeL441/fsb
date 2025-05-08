from machine import Pin, PWM
import time


class Statusled:
    def __init__(self, pin):
        self.led = PWM(Pin(pin))
        self.led.freq(1000)
        self.led.duty_u16(0)  # Off initially
        
    def connected(self):
        # Blink 3 times quickly
        for _ in range(3):
            self.led.duty_u16(65535)  # Full brightness
            time.sleep(0.1)
            self.led.duty_u16(0)
            time.sleep(0.1)
    
    def waiting_for_orders(self):
        # Slow pulse
        self.led.duty_u16(10000)  # Dim
    
    def calibration(self):
        # Quick flash
        self.led.duty_u16(65535)  # Full brightness
        time.sleep(0.05)
        self.led.duty_u16(0)
        time.sleep(0.05)
    
    def collision(self):
        # SOS pattern
        for _ in range(3):
            self.led.duty_u16(65535)
            time.sleep(0.1)
            self.led.duty_u16(0)
            time.sleep(0.1)
        time.sleep(0.2)
        for _ in range(3):
            self.led.duty_u16(65535)
            time.sleep(0.3)
            self.led.duty_u16(0)
            time.sleep(0.1)
        time.sleep(0.2)
        for _ in range(3):
            self.led.duty_u16(65535)
            time.sleep(0.1)
            self.led.duty_u16(0)
            time.sleep(0.1)
    
    def manual_control(self):
        # Fast pulse
        self.led.duty_u16(30000)  # Medium brightness
    
    def loading_animation(self):
        # Breathing effect
        for i in range(0, 65535, 5000):
            self.led.duty_u16(i)
            time.sleep(0.05)
        for i in range(65535, 0, -5000):
            self.led.duty_u16(i)
            time.sleep(0.05)
    
    def reverse(self):
        # Quick double flash
        self.led.duty_u16(65535)
        time.sleep(0.05)
        self.led.duty_u16(0)
        time.sleep(0.05)
        self.led.duty_u16(65535)
        time.sleep(0.05)
        self.led.duty_u16(0)
