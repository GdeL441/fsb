from machine import ADC
import time

class Sensor:
    def __init__(self, pin_sensor, threshold):  # Initialize the sensor
        self.pin = ADC(pin_sensor)
        self.threshold = threshold

    def status(self):  # Returns True or False (when overline)
        return self.pin.read_u16() < self.threshold

    def value(self):  # Returns the current value of the sensor
        return self.pin.read_u16()

    def set_threshold(self, new_threshold):  # Sets the threshold for the sensor
        self.threshold = new_threshold

    def calibrate(self, calibration_threshold):
        # Take multiple readings for more stable calibration
        samples = []
        for _ in range(5):  # Take 5 samples
            samples.append(self.value())
            time.sleep(0.01)  # Short delay between samples
        
        # Use median value for better results
        samples.sort()
        white_value = samples[2]  # Median Value

        threshold = white_value * calibration_threshold
        self.set_threshold(threshold) # Calibrates the sensor
        return threshold # Return the value (used to send it to frontend)
