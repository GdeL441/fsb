from machine import Pin, PWM

class Motor:
    def __init__(self, pin_speed, pin_direction):  # Initialize the motor
        self.speed_pin = PWM(Pin(pin_speed))
        self.speed_pin.freq(1000)  # Set PWM frequency to 1kHz
        self.direction = Pin(pin_direction, Pin.OUT)
        self.stop()  # Initialize in stopped state

    def run(self, speed):  # Speed -100 to 100
        if -100 <= speed < 0:
            self.direction.value(0)  # Set direction pin low for reverse
            duty_cycle = speed_to_duty_cycle(abs(speed))
            self.speed_pin.duty_u16(duty_cycle)
        elif 0 < speed <= 100:
            self.direction.value(1)  # Set direction pin high for forward
            duty_cycle = speed_to_duty_cycle(speed)
            self.speed_pin.duty_u16(duty_cycle)
        else:
            self.stop()

    def stop(self):  # Stop the motor
        self.speed_pin.duty_u16(0)

def speed_to_duty_cycle(speed):
    # Convert speed percentage (0-100) to duty cycle (0-65535)
    duty_cycle = int(speed * 65535 / 100)
    return duty_cycle
