class Collision:
    def __init__(self, threshold_cm, trigger_pin, echo_pin):
        from machine import Pin, time_pulse_us
        import time
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.threshold_cm = threshold_cm
        self.time_pulse_us = time_pulse_us
        
    def detect(self):
        # Send trigger pulse
        self.trigger.value(0)
        time.sleep_us(2)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        
        # Get echo pulse duration
        duration = self.time_pulse_us(self.echo, 1, 30000)
        
        # Calculate distance in cm (speed of sound = 343m/s)
        if duration > 0:
            distance_cm = duration / 58.0
            return distance_cm < self.threshold_cm
        return False
