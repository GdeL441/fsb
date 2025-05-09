from machine import Pin, PWM
import time

class Motor:
    def __init__(self, pin_speed, pin_direction):  # Initialize the motor
        self.speed_pin = PWM(Pin(pin_speed))
        self.speed_pin.freq(1000)  # Set PWM frequency to 1kHz
        self.direction = Pin(pin_direction, Pin.OUT)
        self.current_speed = 0  # Track current speed
        self.current_direction = 1  # Track current direction (1=forward, 0=reverse)
        self.stop()  # Initialize in stopped state

    def run(self, speed):  # Speed -100 to 100
        # Determine target direction and absolute speed
        target_direction = 1 if speed >= 0 else 0
        target_speed = abs(speed)
        
        # If changing direction and not already stopped
        if target_direction != self.current_direction and self.current_speed > 0:
            # Gradually slow down first
            self._decelerate()
            # Then change direction
            self.direction.value(target_direction)
            self.current_direction = target_direction
        else:
            # Just update direction if needed
            self.direction.value(target_direction)
            self.current_direction = target_direction
        
        # Set the new speed
        duty_cycle = speed_to_duty_cycle(target_speed)
        self.speed_pin.duty_u16(duty_cycle)
        self.current_speed = target_speed

    def _decelerate(self):
        # Gradually reduce speed to avoid current spikes
        steps = min(5, self.current_speed)  # Use at most 5 steps
        if steps <= 1:
            # If speed is very low, just stop immediately
            self.speed_pin.duty_u16(0)
            time.sleep(0.01)  # Small delay
            return
            
        step_size = self.current_speed / steps
        for i in range(steps):
            reduced_speed = self.current_speed - (step_size * (i+1))
            if reduced_speed <= 0:
                break
            duty_cycle = speed_to_duty_cycle(reduced_speed)
            self.speed_pin.duty_u16(duty_cycle)
            time.sleep(0.01)  # Small delay between steps
            
        # Ensure we're fully stopped
        self.speed_pin.duty_u16(0)
        time.sleep(0.05)  # Small pause before direction change

    def stop(self):  # Stop the motor
        # Gradually slow down for smoother stopping
        if self.current_speed > 30:
            self._decelerate()
        self.speed_pin.duty_u16(0)
        self.current_speed = 0

def speed_to_duty_cycle(speed):
    # Convert speed percentage (0-100) to duty cycle (0-65535)
    duty_cycle = int(speed * 65535 / 100)
    return duty_cycle
