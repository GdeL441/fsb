# This file runs when the board boots
import machine
import time

# Wait a moment for everything to initialize
time.sleep(1)

try:
    # Import and run your main program
    import code_multithreaded
    print("test")
except Exception as e:
    # If there's an error, print it and flash an LED
    print("Error:", e)
    led = machine.Pin("LED", machine.Pin.OUT)
    for _ in range(10):
        led.toggle()
        time.sleep(0.2)
