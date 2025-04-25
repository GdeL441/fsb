import busio  # type: ignore
import adafruit_us100  # type: ignore


class Collision:
    def __init__(self, threshold, tx, rx):
        uart = busio.UART(tx, rx, baudrate=9600)
        self.us100 = adafruit_us100.US100(uart)
        self.threshold = threshold

    def detect(self):
        print(self.us100.distance)
        print(self.threshold)
        return False # self.us100.distance < self.threshold
