class Motor:
    def __init__(self, pin_forward, pin_backward):
        from machine import Pin, PWM
        self.forward = PWM(Pin(pin_forward))
        self.backward = PWM(Pin(pin_backward))
        self.forward.freq(1000)
        self.backward.freq(1000)
        self.stop()
    
    def run(self, speed):
        # Convert percentage (-100 to 100) to PWM duty cycle (0 to 65535)
        if speed > 0:
            # Forward
            self.backward.duty_u16(0)
            self.forward.duty_u16(int(speed * 655.35))
        elif speed < 0:
            # Backward
            self.forward.duty_u16(0)
            self.backward.duty_u16(int(-speed * 655.35))
        else:
            # Stop
            self.stop()
    
    def stop(self):
        self.forward.duty_u16(0)
        self.backward.duty_u16(0)
