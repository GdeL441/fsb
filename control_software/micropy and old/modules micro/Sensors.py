class Sensor:
    def __init__(self, pin, threshold=12000):
        from machine import ADC
        self.adc = ADC(pin)
        self.threshold = threshold
    
    def value(self):
        # Read raw ADC value (0-65535)
        return self.adc.read_u16()
    
    def status(self):
        # Return True if sensor value is above threshold
        return self.value() > self.threshold
    
    def set_threshold(self, threshold):
        self.threshold = threshold
