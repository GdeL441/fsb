import time
import board
import digitalio
from analogio import AnalogIn

# Initialize sensors and voltage input
# Using GP26, 27 and 28
sensor1 = AnalogIn(board.GP26)
sensor2 = AnalogIn(board.GP27)
sensor3 = AnalogIn(board.GP28)

def get_intensity(pin):
    return pin.value

while True:
    print(f"Sensor 1: {get_intensity(sensor1)}")
    print(f"Sensor 2: {get_intensity(sensor2)}")
    print(f"Sensor 3: {get_intensity(sensor3)}")
    time.sleep(0.5)
