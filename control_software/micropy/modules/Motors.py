from machine import Pin, PWM
import time

class Motor:
    def __init__(self, pin_speed, pin_direction):  # Initialize the motor
        self.speed_pin = PWM(Pin(pin_speed))
        self.speed_pin.freq(1000)  # Set PWM frequency to 1kHz
        self.direction = Pin(pin_direction, Pin.OUT)

        self.current_direction = 1  # Track current direction (1=forward, 0=reverse)
        self.stop()  # Initialize in stopped state

    def run(self, speed):  # Speed -100 to 100
        # Determine target direction and absolute speed
        target_direction = 1 if speed >= 0 else 0
        target_speed = abs(speed)
        # Set the speed and direction
        self.direction.value(target_direction)
        duty_cycle = speed_to_duty_cycle(target_speed)
        self.speed_pin.duty_u16(duty_cycle)

    def stop(self):  # Stop the motor
        self.speed_pin.duty_u16(0)


def speed_to_duty_cycle(speed):
    # Convert speed percentage (0-100) to duty cycle (0-65535)
    duty_cycle = int(speed * 65535 / 100)
    return duty_cycle
