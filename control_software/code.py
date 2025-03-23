import time
import json
import board  # type: ignore
import digitalio  # type: ignore
from analogio import AnalogIn  # type: ignore
import socketpool  # type: ignore
import wifi  # type: ignore
from adafruit_httpserver import Server, Request, Response, GET, Websocket  # type: ignore
from modules import Motors, Sensors, Ultrasonic

# import mdns # type: ignore
# import pwmio # type: ignore
# import ipaddress

# Initialize sensors
# Using GP26, 27 and 28
# 'Sensor.status()' returns True of False based on threshold
L_overline = Sensors.Sensor(board.GP26, 40000)
R_overline = Sensors.Sensor(board.GP27, 45000)
B_overline = Sensors.Sensor(board.GP28, 30000)

# Initialize status sensor (To Be Replaced by RGB Strip)
status_led = digitalio.DigitalInOut(board.LED)
status_led.direction = digitalio.Direction.OUTPUT

# Initaliaze Motors
Motor_Left = Motors.Motor(board.GP2, board.GP3)
Motor_Right = Motors.Motor(board.GP4, board.GP5)

# Initialize Ultrasonic, in cm
collision = Ultrasonic.Collision(5)

# Is the timer currently running, should the car continue making progres on the steps
started = False
# Keep track of the time since next_step called, this will help when turning
time_since_next_step = time.monotonic()

# is the frontend currently monitoring the sensor values
MONITORING_SENSOR = False

# Initialize robot position/heading and grid
robot_pos = {"x": 1, "y": 1}
DIRECTIONS = ["N", "E", "S", "W"]
robot_heading = "N"  # "N" "E" "S" "W"

# Steps the robot should take, later this should be computed at runtime(grid backtracking)
steps = [
    "FORWARD",
    "FORWARD",
    "LEFT",
    "FORWARD",
    "LEFT",
    "FORWARD",
    "LEFT",
    "FORWARD",
    "RIGHT",
]
current_step = 0
# This is required because when the front sensors detect the intersection,
# the car should keep driving until the inteserction is reached
intersection_detected = False

# WiFi configuration
SSID = "Fast Shitbox"
# PASSWORD = "password"
PORT = 80

wifi.radio.start_ap(ssid=SSID)

pool = socketpool.SocketPool(wifi.radio)
server = Server(pool, "/static", debug=True)
websocket = None

# print IP adres
print("My IP address is", wifi.radio.ipv4_address_ap)

# PID Constants
Kp = 1.5  # Proportional gain (adjust for faster/slower correction)
Ki = 0.01  # Integral gain (adjust for minor drifting correction)
Kd = 0.2  # Derivative gain (reduces overshoot)

# Base Speed (PWM Duty Cycle, max 65535)
BASE_SPEED = 30000

# Integral & Derivative Terms
error_sum = 0
last_error = 0


# called when server received new connection
@server.route("/ws", GET)
def connect_client(request: Request):
    global websocket

    # TODO: multiple connections?
    if websocket is not None:
        websocket.close()  # Close any existing connection

    websocket = Websocket(request)

    return websocket


# If there is a connected websocket connection, check if there is a new incoming message
def poll_websocket():
    global started, current_step, error_sum, last_error, MONITORING_SENSOR
    assert websocket != None

    data = websocket.receive(fail_silently=True)
    if data is not None:
        data = json.loads(data)

        if data["action"] == "start":
            started = True
        elif data["action"] == "stop":
            started = False
        elif data["action"] == "reset":
            reset_state()
        elif data["action"] == "monitor_sensor":
            MONITORING_SENSOR = not MONITORING_SENSOR
        # TODO
        elif data["action"] == "move":
            print(data["speedL"], data["speedR"])
        else:
            print("Received other data: ", data)


server.start(port=PORT)
print("Server started, open for websocket connection")


# Reset the cars state, this will be triggered either by the frontend or
# when the car ran into a wall
def reset_state():
    global started, current_step, error_sum, last_error, intersection_detected
    print("Reset state")
    started = False
    stop_motors()
    current_step = 0
    error_sum = 0
    last_error = 0
    intersection_detected = False


def turn_left(right_speed=50):
    Motor_Left.run(-right_speed)
    Motor_Right.run(right_speed)
    print("Turn left")


def turn_right(left_speed=50):
    Motor_Right.run(-left_speed)
    Motor_Left.run(left_speed)
    print("Turn right")


def stop_motors():
    Motor_Right.stop()
    Motor_Left.stop()


# whenever the car advances on the grid, update its position and heading for monitoring
# on the frontend
def update_pos_and_heading():
    global robot_pos, robot_heading

    if steps[current_step] == "FORWARD":
        # Move forward based on the current heading
        if robot_heading == "N":
            robot_pos["y"] -= 1
        elif robot_heading == "S":
            robot_pos["y"] += 1
        elif robot_heading == "E":
            robot_pos["x"] += 1
        elif robot_heading == "W":
            robot_pos["x"] -= 1

    elif steps[current_step] == "RIGHT":
        # Turn 90 degrees to the right (clockwise)
        robot_heading = DIRECTIONS[(DIRECTIONS.index(robot_heading) + 1) % 4]

    elif steps[current_step] == "LEFT":
        # Turn 90 degrees to the left (counterclockwise)
        robot_heading = DIRECTIONS[(DIRECTIONS.index(robot_heading) - 1) % 4]

    data = {
        "action": "position_updated",
        "position": robot_pos,
        "heading": robot_heading,
    }

    if websocket != None:
        websocket.send_message(json.dumps(data), fail_silently=True)


# The car should advance to the next stap defined in the global path
def next_step():
    global current_step, started, time_since_next_step, intersection_detected

    update_pos_and_heading()

    current_step += 1
    time_since_next_step = time.monotonic()

    # Completed parcours
    if current_step == len(steps):
        print("Done, reset")
        reset_state()
        data = {
            "action": "finished ",
        }

        if websocket != None:
            websocket.send_message(json.dumps(data), fail_silently=True)
        return

    data = {"action": "next_step", "step": steps[current_step]}

    if websocket != None:
        websocket.send_message(json.dumps(data), fail_silently=True)


def duty_cycle_to_speed(duty_cycle):
    speed = (duty_cycle * 100) / 65535  # Convert duty cycle back to percentage
    return speed


def send_sensor_values():
    if websocket == None:
        return

    data = {
        "action": "sensor_values",
        "L": L_overline.value(),
        "R": R_overline.value(),
        "B": B_overline.value(),
    }
    websocket.send_message(json.dumps(data), fail_silently=True)


# Main loop
while True:
    # print(f"Sensor left: {L_overline.status()}")
    # print(f"Sensor right: {R_overline.status()}")
    # print(f"Sensor back: {B_overline.status()}")

    if started == True:
        if collision.detect():
            print("Collision Detected! Resetting...")
            reset_state()

        elif steps[current_step] == "FORWARD":
            # If the current step is moving forward, just follow the line until the next intersections
            if L_overline.status() and R_overline.status():
                # Both sensors on line -> intersection detected
                print("Front of car over intersection")
                intersection_detected = True

            if B_overline.status() and intersection_detected == True:
                # A intersections was detected and now we are at the intersection -> move to next step
                print("Car at intersection, go to next step")
                stop_motors()
                time.sleep(0.5)
                next_step()
            else:
                print("Follow line with PID-controller")
                error = (
                    L_overline.value() - R_overline.value()
                ) / 65535.0  # Normalize between -1 and 1
                error_sum += error  # Integral term
                error_derivative = error - last_error  # Derivative term
                last_error = error  # Save error for next loop

                # Anti-windup: Limit integral term
                MAX_INTEGRAL = 1.0
                error_sum = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, error_sum))

                # Compute correction
                correction = (Kp * error) + (Ki * error_sum) + (Kd * error_derivative)

                # Adjust motor speeds
                left_speed = int(BASE_SPEED + (correction * BASE_SPEED))
                right_speed = int(BASE_SPEED - (correction * BASE_SPEED))
                # print(f"Left speed {left_speed} Right speed {right_speed}")
                Motor_Left.run(duty_cycle_to_speed(left_speed))
                Motor_Right.run(duty_cycle_to_speed(right_speed))

        else:
            # The robot is currently turning, wait until back on line before moving to next step
            # To make sure the car actually turned 90 degrees, a minimum turning time is introduced (0.5s), this prevents the car
            # from instantly going to the next step on turns(before actually starting the turn)
            if steps[current_step] == "RIGHT":
                # If the robot is turning right, it should stop turning once the right(TODO: left?) sensor hits the black line
                turn_right()
                if (
                    R_overline.status()
                    and time.monotonic() - time_since_next_step > 0.5
                ):
                    stop_motors()
                    next_step()
            elif steps[current_step] == "LEFT":
                # If the robot is turning left, it should stop turning once the left(TODO: right?) sensor hits the black line
                turn_left()
                if (
                    L_overline.status()
                    and time.monotonic() - time_since_next_step > 0.5
                ):
                    stop_motors()
                    next_step()

    else:

        if MONITORING_SENSOR:
            send_sensor_values()

        stop_motors()

    server.poll()

    if websocket is not None:
        poll_websocket()

    time.sleep(0.05)
