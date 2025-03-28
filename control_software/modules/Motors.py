import pwmio # type: ignore
import digitalio # type: ignore


class Motor:
    def __init__(self, pin_speed, pin_direction):
        self.speed = pwmio.PWMOut(pin_speed, frequency=1000)
        self.direction = digitalio.DigitalInOut(pin_direction)
        self.direction.direction = digitalio.Direction.OUTPUT

    def run(self, speed): # Speed -100 tot 100
        if -100 <= speed < 0:
            self.direction.value = True
            duty_cycle = speed_to_duty_cycle(speed)
            self.speed.duty_cycle = duty_cycle

        elif 0 < speed <= 100:
            self.direction.value = False
            duty_cycle = speed_to_duty_cycle(speed)
            self.speed.duty_cycle = duty_cycle
           

        else: 
            print("Invalid speed")

    def stop(self):
        self.speed.duty_cycle = 0


def speed_to_duty_cycle(speed):
    duty_cycle = int(abs(speed) * 65535 / 100)
    return duty_cycle
