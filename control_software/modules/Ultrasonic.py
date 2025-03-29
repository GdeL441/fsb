import board  # type: ignore
import busio  # type: ignore
import adafruit_us100  # type: ignore


class Collision:
    def __init__(self, threshold):
        uart = busio.UART(board.GP0, board.GP1, baudrate=9600)
        self.us100 = adafruit_us100.US100(uart)
        self.threshold = threshold

    def detect(self):
        return self.us100.distance < self.threshold
