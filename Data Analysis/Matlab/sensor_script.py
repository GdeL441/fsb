import time
import board # type: ignore
import digitalio # type: ignore
from analogio import AnalogIn # type: ignore
import socketpool # type: ignore
import wifi # type: ignore
from adafruit_httpserver import Server, Request, Response, GET, Websocket # type: ignore
import mdns # type: ignore
 
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