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
L_overline = Sensors.Sensor(board.GP28, 12000)
R_overline = Sensors.Sensor(board.GP27, 14000)
B_overline = Sensors.Sensor(board.GP26, 10000)

# Initialize status sensor (To Be Replaced by RGB Strip)
status_led = Statusled.Statusled(board.GP18)

# Initaliaze Motors
Motor_Left = Motors.Motor(board.GP14, board.GP15)
Motor_Right = Motors.Motor(board.GP17, board.GP16)

# Initialize Ultrasonic, in cm
collision = Ultrasonic.Collision(10, board.GP0, board.GP1)

# Initialize servo motor
servo = servo.Servo(
    pwmio.PWMOut(board.GP9, duty_cycle=2**15, frequency=50),
    min_pulse=500,
    max_pulse=2500
)
# Timestamp at which servo was activated, this makes it possible to make the pickup non-blocking
servo_active_time = None

# Is the timer currently running, should the car continue making progres on the steps
started = False


# Keep track if first calibration has been done.
calibrated = False

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

# PID Constants for line following
Kp = 1.5  # Proportional gain (adjust for faster/slower correction)
Ki = 0.01  # Integral gain (adjust for minor drifting correction)
Kd = 0.2  # Derivative gain (reduces overshoot)

# PID Constants for turning
TURN_Kp = 2.0  # Proportional gain for turning
TURN_Ki = 0.05  # Integral gain for turning
TURN_Kd = 0.5  # Derivative gain for turning

# Base Speed (100% = max = 65535)
BASE_SPEED = 30  # %
TURN_SPEED = 30  # %

# Integral & Derivative Terms for line following
error_sum = 0
last_error = 0

# Integral & Derivative Terms for turning
turn_error_sum = 0
turn_last_error = 0

# Target heading for turning PID
target_heading = 0  # 0 = straight, positive = right, negative = left

# Add a throttling mechanism for WebSocket messages
last_message_time = time.monotonic()
MIN_MESSAGE_INTERVAL = 0.05  # Minimum time between messages (50ms)

# Improve the WebSocket send function with throttling and error handling
def send_websocket_message(data, important=False):
    """
    Send a message through the WebSocket with throttling and error handling.
    
    Args:
        data: Dictionary to send as JSON
        important: If True, bypass throttling for critical messages
    """
    global websocket, last_message_time
    
    if websocket is None:
        return False
    
    current_time = time.monotonic()
    
    # Skip non-important messages if we're sending too frequently
    if not important and (current_time - last_message_time < MIN_MESSAGE_INTERVAL):
        return False
    
    try:
        # Convert to JSON only once
        json_data = json.dumps(data)
        websocket.send_message(json_data, fail_silently=True)
        last_message_time = current_time
        return True
    except Exception as e:
        print(f"Error sending WebSocket message: {e}")
        return False




# called when server received new connection
@server.route("/ws", GET)
def connect_client(request: Request):
    global websocket

    # TODO: multiple connections?
    if websocket is not None:
        websocket.close()  # Close any existing connection

    websocket = Websocket(request)

    # Once a new websocket connects, we play a nice animation and we send all of the constants for PID, speed,
    # and thresholds to the frontend.
    status_led.connected()
    # data = {
    #     "action": "setup",
    #     "speed": BASE_SPEED,
    #     "turnSpeed": TURN_SPEED,
    #     "P": Kp,
    #     "I": Ki,
    #     "D": Kd,
    #     "L": L_overline.threshold,
    #     "R": R_overline.threshold,
    #     "B": B_overline.threshold,
    # }

    # message_queue.append(json.dumps(data))
    return websocket
# Send al the data, todo

# If there is a connected websocket connection, check if there is a new incoming message
def poll_websocket():
    global started, current_step, error_sum, last_error, MONITORING_SENSOR, Kp, Ki, Kd, BASE_SPEED, TURN_SPEED, steps, robot_pos, robot_heading, MANUAL_CONTROL, MANUAL_CONTROL_SPEEDS, green_towers, websocket, time_since_next_step
    
    if websocket is None:
        return

    try:
        data = websocket.receive(fail_silently=True)
        if data is not None:
            try:
                data = json.loads(data)

                if data["action"] == "start":
                    time_since_next_step = time.monotonic()
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

                    print(f"start data {data}")
                    if data.get("thresholds") != None: 
                        L_overline.set_threshold(data["thresholds"]["L"])
                        R_overline.set_threshold(data["thresholds"]["R"])
                        B_overline.set_threshold(data["thresholds"]["B"])

                    if data["speed"] != None:
                        speed = max(min(100, data["speed"]), 0)
                        BASE_SPEED = speed
                    if data.get("turnSpeed") != None:
                        turn_speed = max(min(100, data["turnSpeed"]), 0)
                        TURN_SPEED = turn_speed
                        

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
                    print("update base and turn speed", data)
                    speed = max(min(100, data["speed"]), 0)
                    turn_speed = max(min(100, data["turnSpeed"]), 0)
                    TURN_SPEED = turn_speed
                    BASE_SPEED = speed
                elif data["action"] == "manual_control":
                    started = False
                    MANUAL_CONTROL = not MANUAL_CONTROL
                elif data["action"] == "manual_control_speeds":
                    if not MANUAL_CONTROL:
                        return

                    MANUAL_CONTROL_SPEEDS = data["speeds"]
                elif data["action"] == "arm_up":
                    if not MANUAL_CONTROL:
                        return

                    servo.angle = 0
                elif data["action"] == "arm_down":
                    if not MANUAL_CONTROL:
                        return

                    servo.angle = 175
                elif data["action"] == "calibrate":
                    print("Calibrating sensors...")
                    # Use a slightly higher threshold for better line detection
                    calibrate_all(threshold=0.9)
                else:
                    print("Received other data: ", data)
            except (ValueError, KeyError) as e:
                print(f"Error processing WebSocket data: {e}")
    except Exception as e:
        print(f"WebSocket error: {e}")
        # If there's an error with the websocket, close it to allow reconnection
        try:
            websocket.close()
        except:
            pass
        websocket = None


server.start(port=PORT)
print("Server started, open for websocket connection")


# Reset the cars state, this will be triggered either by the frontend or
# when the car ran into a wall
def reset_state():
    global started, current_step, error_sum, last_error, intersection_detected, intersection_detection_time, robot_pos, robot_heading, green_towers, servo_active_time
    print("Reset state")
    started = False
    stop_motors()
    current_step = 0
    error_sum = 0
    last_error = 0
    intersection_detected = False
    intersection_detection_time = None
    robot_pos = {"x": 6, "y": 0}
    robot_heading = "N"  # "N" "E" "S" "W"
    green_towers = []
    servo_active_time = None # Ook nog servos naar 0° zetten of gebeurt dit direct?
    servo.angle = 175


# Turn the robot to the left with a given speed, used on intersection.
# Defaults to turn_speed
def turn_left():
    Motor_Left.run(-TURN_SPEED)
    Motor_Right.run(TURN_SPEED)


# Turn the robot to the right with a given speed, used on intersection.
# Defaults to turn_speed
def turn_right():
    Motor_Right.run(-TURN_SPEED)
    Motor_Left.run(TURN_SPEED)


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
    
    # Position updates are important
    send_websocket_message(data, important=True)

# Return the next step in the path or None
def get_next_step():
    if current_step + 1 == len(steps): 
        return None

    return steps[current_step + 1]


# The car should advance to the next stap defined in the global path
def next_step():
    global current_step, started, time_since_next_step, intersection_detected

    # Calculate and update the new position and heading, since it has completed a step.
    update_pos_and_heading()

    # If there is a green tower on the current position, use the arm to pickup the tower.
    # Maybe we change this to a pickup command from the backtracking algorithm?
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
        send_websocket_message(data, important=True)
        return

    data = {"action": "next_step", "step": steps[current_step]}
    send_websocket_message(data, important=True)


# Pick up a tower by turning the servo 180 degrees, keep track of this Timestamp
# so after a specified duration, the arm moves back down, this is done so this
# code doesn't block the car from making progres
def pickup():
    global servo_active_time
    servo.angle = 0
    servo_active_time = time.monotonic()


# Convert duty_cycle (0-65535) to % speeds (0-100)
# def duty_cycle_to_speed(duty_cycle):
#     speed = (duty_cycle * 100) / 65535  # Convert duty cycle back to percentage
#     return speed


# Used for calibrating the LDRs from the app. Only in monitoring mode.
def send_sensor_values():
    assert MONITORING_SENSOR
    
    data = {
        "action": "sensor_values",
        "L": L_overline.value(),
        "R": R_overline.value(),
        "B": B_overline.value(),
    }
    # Sensor values are less critical, so no important flag
    send_websocket_message(data)


# If the car is in manual control mode, use the speeds received from the app
# to turn the motors.
def manual_control():
    assert MANUAL_CONTROL
    left, right = MANUAL_CONTROL_SPEEDS["left"], MANUAL_CONTROL_SPEEDS["right"]
    Motor_Left.run(left)
    Motor_Right.run(right)
    if ( left < 0 ) and ( right < 0 ):
        status_led.reverse()


# Utility function to search the first occurence in list with lambda function, if not found
# return None.
def find(lst, fn):
    for x in lst:
        if fn(x):
            return x
    return None


def calibrate_sensor(sensor, calibration_threshold = 0.8):
    white_value = sensor.value()
    threshold = white_value * calibration_threshold
    sensor.set_threshold(threshold)
    return threshold

def calibrate_all(threshold = 0.8, send = True):
    r_threshold = calibrate_sensor(R_overline, threshold)
    l_threshold = calibrate_sensor(L_overline, threshold)
    b_threshold = calibrate_sensor(B_overline, threshold)
    status_led.calibration()
    print("All sensors have been calibrated")
    
    # Send the new threshold values to the frontend
    if send:
        data = {
            "action": "thresholds_updated",
            "thresholds": {
                "L": l_threshold,
                "R": r_threshold,
                "B": b_threshold
            }
        }
        send_websocket_message(data, important=True)

# Main loop
while True:
    
    
    if not calibrated: # This will calibrate the sensors when booting the car. Can be changed to when a connection has been established.
        calibrate_all(0.9, False)
        calibrated = True
        
    # print(f"Sensor left: {L_overline.status()}")
    # print(f"Sensor right: {R_overline.status()}")
    # print(f"Sensor back: {B_overline.status()}")
    if started == True:
        
        if collision.detect():
            print("Collision Detected! Resetting...")
            status_led.collision()
            reset_state()

        if steps[current_step] == "FORWARD":
            # If the current step is moving forward, just follow the line until the next intersections
            if L_overline.status() and R_overline.status():
                # Both sensors on line -> intersection detected
                intersection_detected = True
                #time_since_next_step = time.monotonic() # Ik heb deze regel toegevoegd. Misschien helpt dit het intersection detecten???

            if (B_overline.status() and intersection_detected == True
                    and time.monotonic() - time_since_next_step > 0.9
                ):
                # A intersections was detected and now we are at the intersection -> move to next step
                # Only stop the motors if the path is finished or the next step is not FORWARD.
                possible_next_step = get_next_step()
                if possible_next_step == None or possible_next_step != "FORWARD":
                    stop_motors()

                intersection_detcted = False
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
                Motor_Left.run(left_speed)
                Motor_Right.run(right_speed)

        else:
            # The robot is currently turning, wait until back on line before moving to next step
            # To make sure the car actually turned 90 degrees, a minimum turning time is introduced (0.5s), this prevents the car
            # from instantly going to the next step on turns(before actually starting the turn)
            if steps[current_step] == "RIGHT":
                # If the robot is turning right, it should stop turning once the right(TODO: left?) sensor hits the black line
                turn_right()
                if (
                    R_overline.status()
                    and time.monotonic() - time_since_next_step > 0.9
                ):
                    stop_motors()
                    next_step()
            elif steps[current_step] == "LEFT":
                # If the robot is turning left, it should stop turning once the left(TODO: right?) sensor hits the black line
                turn_left()
                if (
                    L_overline.status()
                    and time.monotonic() - time_since_next_step > 0.9
                ):
                    stop_motors()
                    next_step()
    else:
        if MANUAL_CONTROL:
            manual_control()
            status_led.manual_control() 
        elif MONITORING_SENSOR:
            send_sensor_values()
        else:
            stop_motors()
            status_led.waiting_for_orders()

    if servo_active_time != None:
        status_led.collection()
        # The servo is activated to move up, if it has been in this 'up' state for longer than 1 second,
        # move the servo back down
        if time.monotonic() - servo_active_time > 0.7:
            servo.angle = 175
            servo_active_time = None
    elif started:
        status_led.next_object()

    # Polling HTTP server
    server.poll()

    if websocket is not None:
        # If there is a websocket connection, send all queued messages (setup data, ...)
        while len(message_queue) > 0:
            msg = message_queue.pop(0)
            websocket.send_message(json.dumps(msg), fail_silently=True)

        poll_websocket()
    else:
        status_led.loading_animation()


