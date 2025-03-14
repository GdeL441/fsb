import time
import board
import digitalio
from analogio import AnalogIn
import socketpool
import wifi
from adafruit_httpserver import Server, Request, Response, GET, Websocket
import mdns

# Initialize sensors 
# Using GP26, 27 and 28
sensorL = AnalogIn(board.GP26)
sensorR = AnalogIn(board.GP27)
sensorB = AnalogIn(board.GP28)

while True:
    print(f"{sensorB.value},")
    #print(f"Sensor Rechts: {sensorR.value}")
    #print(f"Sensor Back: {sensorB.value}")
    time.sleep(0.05)
