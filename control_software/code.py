import time
import json
import pwmio  # type: ignore
import board  # type: ignore
from analogio import AnalogIn  # type: ignore
import socketpool  # type: ignore
import wifi  # type: ignore
from adafruit_httpserver import Server, Request, Response, GET, Websocket  # type: ignore
from modules import Motors, Sensors, Ultrasonic, Statusled
from adafruit_motor import servo  # type: ignore

# Initialize sensors
# Using GP26, 27 and 28 (To be changed (NEW PCB))
# 'Sensor.status()' returns True of False based on threshold
L_overline = Sensors.Sensor(board.GP26, 12000)
R_overline = Sensors.Sensor(board.GP27, 14000)
B_overline = Sensors.Sensor(board.GP28, 10000)

# Initialize status sensor (To Be Replaced by RGB Strip)
status_led = Statusled.Statusled(board.GP18)

# Initaliaze Motors
Motor_Left = Motors.Motor(board.GP14, board.GP15)
Motor_Right = Motors.Motor(board.GP17, board.GP16)

# Initialize Ultrasonic, in cm
collision = Ultrasonic.Collision(10, board.GP0, board.GP1)

# Initialize servo motor
servo = servo.Servo(pwmio.PWMOut(board.GP9, duty_cycle=2**15, frequency=50))
# Timestamp at which servo was activated, this makes it possible to make the pickup non-blocking
servo_active_time = None

# Is the timer currently running, should the car continue making progres on the steps
started = False

# Keep track of the time since next_step called, this will help when turning
time_since_next_step = time.monotonic()

# is the frontend currently monitoring the sensor values
MONITORING_SENSOR = False

# Does the frontend ask for manual control
MANUAL_CONTROL = False
MANUAL_CONTROL_SPEEDS = {"left": 0, "right": 0}

# Initialize robot position/heading and grid
robot_pos = {"x": 6, "y": 0}
DIRECTIONS = ["N", "E", "S", "W"]
robot_heading = "N"  # "N" "E" "S" "W"
green_towers = []  # Location of green towers

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
message_queue = []

# print IP adres
print("My IP address is", wifi.radio.ipv4_address_ap)

# PID Constants
Kp = 1.5  # Proportional gain (adjust for faster/slower correction)
Ki = 0.01  # Integral gain (adjust for minor drifting correction)
Kd = 0.2  # Derivative gain (reduces overshoot)

# Base Speed (PWM Duty Cycle, max 65535)
BASE_SPEED = 20000
TURN_SPEED = 20000

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

    # Once a new websocket connects, we send all of the constants for PID, speed,
    # and thresholds to the frontend.
    data = {
        "action": "setup",
        "speed": BASE_SPEED,
        "turn_speed": TURN_SPEED,
        "P": Kp,
        "I": Ki,
        "D": Kd,
        "L": L_overline.threshold,
        "R": R_overline.threshold,
        "B": B_overline.threshold,
    }

    message_queue.append(json.dumps(data))
    return websocket


# If there is a connected websocket connection, check if there is a new incoming message
def poll_websocket():
    global started, current_step, error_sum, last_error, MONITORING_SENSOR, Kp, Ki, Kd, BASE_SPEED, TURN_SPEED, steps, robot_pos, robot_heading, MANUAL_CONTROL, MANUAL_CONTROL_SPEEDS, green_towers
    assert websocket != None

    data = websocket.receive(fail_silently=True)
    if data is not None:
        data = json.loads(data)

        if data["action"] == "start":
            MANUAL_CONTROL = False
            if (
                data["path"] != None
                and data["startX"] != None
                and data["startY"] != None
                and data["heading"] != None
                and data["green_towers"] != None
            ):
                steps = data["path"]
                robot_pos = {"x": data["startX"], "y": data["startY"]}
                robot_heading = data["heading"]
                green_towers = data["green_towers"]

            started = True
        elif data["action"] == "stop":
            started = False
        elif data["action"] == "reset":
            reset_state()
        elif data["action"] == "monitor_sensor":
            MONITORING_SENSOR = not MONITORING_SENSOR
        elif data["action"] == "set_threshold":
            print("update sensor thresholds", data)
            L_overline.set_threshold(data["L"])
            R_overline.set_threshold(data["R"])
            B_overline.set_threshold(data["B"])
        elif data["action"] == "update_pid":
            print("update PID parameters", data)
            Kp = data["P"]
            Ki = data["I"]
            Kd = data["D"]
        elif data["action"] == "update_speed":
            print("update base speed", data)
            speed = max(min(65535, data["speed"]), 0)
            BASE_SPEED = speed
        elif data["action"] == "update_turn_speed":
            print("update turn base speed", data)
            speed = max(min(65535, data["speed"]), 0)
            TURN_SPEED = speed
        elif data["action"] == "manual_control":
            started = False
            MANUAL_CONTROL = not MANUAL_CONTROL
        elif data["action"] == "manual_control_speeds":
            if not MANUAL_CONTROL:
                return

            MANUAL_CONTROL_SPEEDS = data["speeds"]
        else:
            print("Received other data: ", data)


server.start(port=PORT)
print("Server started, open for websocket connection")


# Reset the cars state, this will be triggered either by the frontend or
# when the car ran into a wall
def reset_state():
    global started, current_step, error_sum, last_error, intersection_detected, robot_pos, robot_heading, green_towers, servo_active_time
    print("Reset state")
    started = False
    stop_motors()
    current_step = 0
    error_sum = 0
    last_error = 0
    intersection_detected = False
    robot_pos = {"x": 6, "y": 0}
    robot_heading = "N"  # "N" "E" "S" "W"
    green_towers = []
    servo_active_time = None


# Turn the robot to the left with a given duty_cycle speed, used on intersection.
# Defaults to TURN_SPEED
def turn_left(duty_cycle=TURN_SPEED):
    turning_speed = duty_cycle_to_speed(duty_cycle)
    print(f"Turn left, speed={turning_speed}")
    Motor_Left.run(-turning_speed)
    Motor_Right.run(turning_speed)


# Turn the robot to the right with a given duty_cycle speed, used on intersection.
# Defaults to TURN_SPEED
def turn_right(duty_cycle=TURN_SPEED):
    turning_speed = duty_cycle_to_speed(duty_cycle)
    print(f"Turn right, speed={turning_speed}")
    Motor_Right.run(-turning_speed)
    Motor_Left.run(turning_speed)


def stop_motors():
    Motor_Right.stop()
    Motor_Left.stop()


# whenever the car advances on the grid, update its position and heading for monitoring
# on the frontend
def update_pos_and_heading():
    global robot_pos, robot_heading
    # print(
    #     "update pos and heading before", robot_pos, robot_heading, steps[current_step]
    # )

    if steps[current_step] == "FORWARD":
        # Move forward based on the current heading
        if robot_heading == "N":
            robot_pos["y"] += 1
        elif robot_heading == "S":
            robot_pos["y"] -= 1
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
    print("update pos and heading after", robot_pos, robot_heading, steps[current_step])

    if websocket != None:
        websocket.send_message(json.dumps(data), fail_silently=True)


# The car should advance to the next stap defined in the global path
def next_step():
    global current_step, started, time_since_next_step, intersection_detected

    # Calculate and update the new position and heading, since it has completed a step.
    update_pos_and_heading()

    # If there is a green tower on the current position, use the arm to pickup the tower.
    tower = find(
        green_towers, lambda t: t["x"] == robot_pos["x"] and t["y"] == robot_pos["y"]
    )
    if tower != None:
        print("Pick up item with servo arm")
        pickup()
        green_towers.remove(tower)

    current_step += 1
    time_since_next_step = time.monotonic()

    # Completed parcours
    if current_step == len(steps):
        print("Done, reset")
        reset_state()
        data = {
            "action": "finished",
        }

        if websocket != None:
            websocket.send_message(json.dumps(data), fail_silently=True)
        return

    data = {"action": "next_step", "step": steps[current_step]}

    if websocket != None:
        websocket.send_message(json.dumps(data), fail_silently=True)


# Pick up a tower by turning the servo 180 degrees, keep track of this Timestamp
# so after a specified duration, the arm moves back down, this is done so this
# code doesn't block the car from making progres
def pickup():
    global servo_active_time
    servo.angle = 180
    servo_active_time = time.monotonic()


# Convert duty_cycle (0-65535) to % speeds (0-100)
def duty_cycle_to_speed(duty_cycle):
    speed = (duty_cycle * 100) / 65535  # Convert duty cycle back to percentage
    return speed


# Used for calibrating the LDRs from the app. Only in monitoring mode.
def send_sensor_values():
    assert MONITORING_SENSOR
    if websocket == None:
        return

    data = {
        "action": "sensor_values",
        "L": L_overline.value(),
        "R": R_overline.value(),
        "B": B_overline.value(),
    }
    websocket.send_message(json.dumps(data), fail_silently=True)


# If the car is in manual control mode, use the speeds received from the app
# to turn the motors.
def manual_control():
    assert MANUAL_CONTROL
    left, right = MANUAL_CONTROL_SPEEDS["left"], MANUAL_CONTROL_SPEEDS["right"]
    print(f"Manual control: left={left} right={right}")
    Motor_Left.run(left)
    Motor_Right.run(right)


# Utility function to search the first occurence in list with lambda function, if not found
# return None.
def find(lst, fn):
    return next((x for x in lst if fn(x)), None)


# Main loop
while True:
    # print(f"Sensor left: {L_overline.status()}")
    # print(f"Sensor right: {R_overline.status()}")
    # print(f"Sensor back: {B_overline.status()}")
    if started == True:
        # status_led.next_object() # Invalid State
        # if collision.detect():
        #    print("Collision Detected! Resetting...")
        #    reset_state()

        if steps[current_step] == "FORWARD":
            # If the current step is moving forward, just follow the line until the next intersections
            if L_overline.status() and R_overline.status():
                # Both sensors on line -> intersection detected
                print("Front of car over intersection")
                intersection_detected = True

            if B_overline.status() and intersection_detected == True:
                # A intersections was detected and now we are at the intersection -> move to next step
                print("Car at intersection, go to next step")
                # Todo if the next step is also forward, don't stop the motors here, just keep driving
                stop_motors()
                intersection_detected = False
                next_step()
            else:
                # print("Follow line with PID-controller")
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

        if MANUAL_CONTROL:
            manual_control()

        if MONITORING_SENSOR:
            send_sensor_values()

        stop_motors()
        status_led.turn_off()

    if servo_active_time != None:
        # The servo is activated to move up, if it has been in this 'up' state for longer than 1 second,
        # move the servo back down
        if time.monotonic() - servo_active_time > 1:
            servo.angle = 0
            servo_active_time = None

    server.poll()

    if websocket is not None:
        # If there is a websocket connection, send all queued messages (setup data, ...)
        while msg := message_queue.pop(0):
            websocket.send_message(json.dumps(msg), fail_silently=True)

        poll_websocket()

    time.sleep(0.01)
