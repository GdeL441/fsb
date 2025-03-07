import time
import pwmio
import ipaddress
import board
from digitalio import Direction, DigitalInOut
from analogio import AnalogIn
import socketpool
import wifi
from adafruit_httpserver import Server, Request, Response, GET, Websocket
import mdns

# Initialize sensors
# Using GP26, 27 and 28
sensorL = {"pin": AnalogIn(board.GP26), "threshold": 25000}
sensorR = {"pin": AnalogIn(board.GP27), "threshold": 25000}
sensor3 = {"pin": AnalogIn(board.GP28), "threshold": 25000}

# Initialize status sensor
status_led = DigitalInOut(board.LED)
status_led.direction = Direction.OUTPUT

# Define motor driver pins
MOTOR_LEFT_PWM = pwmio.PWMOut(board.GP2, frequency=1000)  # Left motor speed control
MOTOR_LEFT_IN1 = DigitalInOut(board.GP3)  # Left motor forward
MOTOR_LEFT_IN2 = DigitalInOut(board.GP4)  # Left motor backward

MOTOR_RIGHT_PWM = pwmio.PWMOut(board.GP5, frequency=1000)  # Right motor speed control
MOTOR_RIGHT_IN1 = DigitalInOut(board.GP6)  # Right motor forward
MOTOR_RIGHT_IN2 = DigitalInOut(board.GP7)  # Right motor backward

# Set IN1 and IN2 as outputs
for pin in [MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2]:
    pin.direction = Direction.OUTPUT


# Initialize robot position/heading and grid
robot_pos = {"x": 1, "y": 1}
robot_heading = "N"  # "N" "E" "S" "W"

# Steps the robot should take, later this should be computed at runtime(grid backtracking)
steps = ["FORWARD", "LEFT", "FORWARD", "RIGHT", "FORWARD"]
current_step = 0
intersection_detected = False

# WiFi configuration
SSID = "PICO-FSB-502"
# PASSWORD = "password"  #Verander voor veiligheidsredenen, wat is veiligheid?
PORT = 80

# Initialize mDNS
mdns_server = mdns.Server(wifi.radio)
mdns_server.hostname = "fast-shitbox"
mdns_server.advertise_service(service_type="_http", protocol="_tcp", port=PORT)
print("mDNS advertised: _http._tcp, hostname='fast-shitbox'")

#  set static IP address
ipv4 = ipaddress.IPv4Address("192.168.1.42")
netmask = ipaddress.IPv4Address("255.255.255.0")
gateway = ipaddress.IPv4Address("192.168.1.1")
wifi.radio.set_ipv4_address(ipv4=ipv4, netmask=netmask, gateway=gateway)

wifi.radio.start_ap(ssid=SSID)

# print IP adres
print("My IP address is", wifi.radio.ipv4_address_ap)

pool = socketpool.SocketPool(wifi.radio)
server = Server(pool, "/static", debug=True)
websocket = None


# called when server received new connection
@server.route("/ws", GET)
def connect_client(request: Request):
    global websocket

    # TODO: multiple connections?
    if websocket is not None:
        websocket.close()  # Close any existing connection

    websocket = Websocket(request)

    return websocket


def poll_websocket():
    assert websocket != None

    data = websocket.receive(fail_silently=True)
    if data is not None:
        print(data)
        websocket.send_message(data, fail_silently=True)


server.start(port=PORT)
print("Server started, open for websocket connection")


def over_line(sensor):
    return sensor["pin"].value < sensor["threshold"]


def move_forward():
    print("Forward")
    turn_left(70)
    turn_right(70)


def turn_left(right_speed = 50):
    MOTOR_RIGHT_IN1.value = True
    MOTOR_RIGHT_IN2.value = False
    MOTOR_LEFT_IN1.value = False
    MOTOR_LEFT_IN2.value = False
    MOTOR_RIGHT_PWM.duty_cycle = int((right_speed / 100) * 65535)
    print("Turn left")


def turn_right(left_speed = 50):
    MOTOR_LEFT_IN1.value = True
    MOTOR_LEFT_IN2.value = False
    MOTOR_RIGHT_IN1.value = False
    MOTOR_RIGHT_IN2.value = False
    MOTOR_LEFT_PWM.duty_cycle = int((left_speed / 100) * 65535)
    print("Turn left")


def stop_motors():
    MOTOR_RIGHT_IN1.value = False
    MOTOR_RIGHT_IN2.value = False
    MOTOR_RIGHT_PWM.duty_cycle = 0
    MOTOR_LEFT_IN1.value = False
    MOTOR_LEFT_IN2.value = False
    MOTOR_LEFT_PWM.duty_cycle = 0
    print("Stop")


def next_step():
    global current_step
    current_step += 1


# Main loop
while True:
    print(f"Sensor left: {over_line(sensorL)}")
    print(f"Sensor right: {over_line(sensorR)}")
    print(f"Sensor back: {over_line(sensor3)}")

    # TODO
    if current_step + 1 != len(steps):
        if steps[current_step] == "FORWARD":
            # If the current step is moving forward, just follow the line until the next intersections

            if not over_line(sensorL) and not over_line(sensorR):
                # TODO: is if statement necessary?
                move_forward()
            if over_line(sensorL) and over_line(sensorR):
                # Both sensors on line -> intersection detected
                intersection_detected = True
                move_forward()
            elif over_line(sensorL):
                # Left sensors on line -> robot should correct by steering left
                turn_left()
            elif over_line(sensorR):
                # Right sensors on line -> robot should correct by steering right
                turn_right()
            else: 
                print("lost")
                stop_motors()

            if over_line(sensor3) and intersection_detected:
                # A intersections was detected and now we are at the intersection -> move to next step
                intersection_detected = False
                stop_motors()
                next_step()

        else:
            # The robot is currently turning, wait until back on line before moving to next step
            # TODO: add reference time so this doesn't trigger before the turn even started
            if steps[current_step] == "RIGHT" and over_line(sensorL):
                # If the robot is turning right, it should stop turning once the left sensor hits the black line
                stop_motors()
                next_step()
            elif steps[current_step] == "LEFT" and over_line(sensorR):
                # If the robot is turning left, it should stop turning once the right sensor hits the black line
                stop_motors()
                next_step()

    server.poll()

    if websocket is not None:
        poll_websocket()

    time.sleep(0.5)
