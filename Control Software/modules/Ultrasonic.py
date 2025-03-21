import time # type: ignore
import board # type: ignore
import busio # type: ignore
import adafruit_us100 # type: ignore

uart = busio.UART(board.GP0, board.GP1, baudrate=9600)
# Create a US-100 module instance.
us100 = adafruit_us100.US100(uart)

while True:
    print("-----")
    print(f"Temperature: {us100.temperature}°C")
    print(f"Distance: {us100.distance} cm")
    time.sleep(0.5)