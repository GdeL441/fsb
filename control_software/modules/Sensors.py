import analogio  # type: ignore


class Sensor:
    def __init__(self, pin_sensor, threshold):  # Initialize the sensor
        self.pin = analogio.AnalogIn(pin_sensor)
        self.threshold = threshold

    def status(self):  # Returns True or False (when overline)
        return self.pin.value < self.threshold

    def value(self):  # Returns the current value of the sensor
        return self.pin.value

    def set_threshold(self, new_threshold):  # Sets the threshold for the sensor
        self.threshold = new_threshold
