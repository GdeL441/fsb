import pwmio # type: ignore
import analogio # type: ignore

class Sensor:
    def __init__(self, pin_sensor, threshold):
        self.pin = analogio.AnalogIn(pin_sensor)
        self.threshold = threshold

    def status(self): # Speed -100 tot 100
        return self.pin.value < self.threshold

    def value(self):  
        return self.pin.value 

    def set_threshold(self, new_threshold):
        self.threshold = new_threshold
