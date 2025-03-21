import time
import pwmio # type: ignore
import ipaddress
import json
import board # type: ignore
import digitalio # type: ignore
from analogio import AnalogIn # type: ignore
import socketpool # type: ignore
import wifi # type: ignore
from adafruit_httpserver import Server, Request, Response, GET, Websocket # type: ignore
import mdns # type: ignore
from modules import Motors, Sensors, Ultrasonic

# Initialize sensors
# Using GP26, 27 and 28
# Returns True of False
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

started = False
# Keep track of the time since next_step called, this will help when turning
time_since_next_step = time.monotonic()

# Initialize robot position/heading and grid
robot_pos = {"x": 1, "y": 1}
robot_heading = "N"  # "N" "E" "S" "W"

# Steps the robot should take, later this should be computed at runtime(grid backtracking)
# steps = ["FORWARD", "LEFT", "FORWARD", "RIGHT", "FORWARD"]
steps = ["FORWARD", "FORWARD", "LEFT", "FORWARD", "LEFT", "FORWARD", "LEFT", "FORWARD", "RIGHT"]
current_step = 0
intersection_detected = False

# WiFi configuration
SSID = "Fast Shitbox"
# PASSWORD = "password"  #Verander voor veiligheidsredenen
PORT = 80

wifi.radio.start_ap(ssid=SSID)

pool = socketpool.SocketPool(wifi.radio)
server = Server(pool, "/static", debug=True)
websocket = None

# print IP adres
print("My IP address is", wifi.radio.ipv4_address_ap)


# PID Constants (Tune these)
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


def poll_websocket():
    global started, current_step, error_sum, last_error
    assert websocket != None

    data = websocket.receive(fail_silently=True)
    if data is not None:
        data = json.loads(data)

        if data["action"] == "start":
            started = True
        elif data["action"] == "stop":
            started = False
        elif data["action"] == "reset":
            started = False
            stop_motors()
            current_step = 0
            error_sum = 0
            last_error = 0
        elif data["action"] == "move":
            print(data["speedL"], data["speedR"])
        else:
            print("Received other data: ", data)
        


server.start(port=PORT)
print("Server started, open for websocket connection")


def move_forward(speed = 70):
    Motor_Left.run(speed)
    Motor_Right.run(speed)
    print("Forward")

def turn_left(right_speed = 50):
    Motor_Left.run(-right_speed)
    Motor_Right.run(right_speed)
    print("Turn left")


def turn_right(left_speed = 50):
    Motor_Right.run(-left_speed)
    Motor_Left.run(left_speed)
    print("Turn right")


def stop_motors():
    Motor_Right.stop()
    Motor_Left.stop()


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
        directions = ["N", "E", "S", "W"]
        robot_heading = directions[(directions.index(robot_heading) + 1) % 4]

    elif steps[current_step] == "LEFT":
        # Turn 90 degrees to the left (counterclockwise)
        directions = ["N", "E", "S", "W"]
        robot_heading = directions[(directions.index(robot_heading) - 1) % 4]

    data = {
        "action": "position_updated",
        "position": robot_pos,
        "heading": robot_heading
    }

    if websocket != None: 
        websocket.send_message(json.dumps(data), fail_silently=True)

def next_step():
    global current_step, started, time_since_next_step, intersection_detected

    update_pos_and_heading()

    current_step += 1
    time_since_next_step = time.monotonic()


    # Completed parcours
    if current_step == len(steps):
        started = False
        current_step = 0 
        intersection_detected = False
        print("Done, reset")
        data = {
            "action": "finished ", 
        }

        if websocket != None: 
            websocket.send_message(json.dumps(data), fail_silently=True)
        return

    data = {
     "action": "next_step", 
     "step": steps[current_step] 
    }

    if websocket != None: 
        websocket.send_message(json.dumps(data), fail_silently=True)


def duty_cycle_to_speed(duty_cycle):
    speed = (duty_cycle * 100) / 65535  # Convert duty cycle back to percentage
    return speed



# Main loop
while True:
    #print(f"Sensor left: {L_overline.status()}")
    #print(f"Sensor right: {R_overline.status()}")
    #print(f"Sensor back: {B_overline.status()}")

    if started == True:
        if collision.detect():
            started = False
            stop_motors()
            current_step = 0
            error_sum = 0
            last_error = 0
            print("Collision Detected!")

        elif steps[current_step] == "FORWARD":
            # If the current step is moving forward, just follow the line until the next intersections
            if L_overline.status() and R_overline.status():
                # Both sensors on line -> intersection detected
                intersection_detected = True
                print("Front over intersection")
                #move_forward()

            if B_overline.status() and intersection_detected == True:
                # A intersections was detected and now we are at the intersection -> move to next step
                print("Whole car over intersection!")
                stop_motors()
                time.sleep(0.5)
                next_step()
            else: 
                print("Just follow line with PID-controller")
                error = (L_overline.value() - R_overline.value()) / 65535.0  # Normalize between -1 and 1
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
                #print(f"Left speed {left_speed} Right speed {right_speed}")
                Motor_Left.run(duty_cycle_to_speed(left_speed))
                Motor_Right.run(duty_cycle_to_speed(right_speed))


        else:
            # The robot is currently turning, wait until back on line before moving to next step
            # TODO: add reference time so this doesn't trigger before the turn even started
            if steps[current_step] == "RIGHT":
                # If the robot is turning right, it should stop turning once the left sensor hits the black line
                turn_right()
                if R_overline.status() and time.monotonic() - time_since_next_step > 0.5:
                    stop_motors()
                    next_step()
            elif steps[current_step] == "LEFT" :
                # If the robot is turning left, it should stop turning once the right sensor hits the black line
                turn_left()
                if L_overline.status() and time.monotonic() - time_since_next_step > 0.5:
                    stop_motors()
                    next_step()

    else:
        stop_motors()

    server.poll()

    if websocket is not None:
        poll_websocket()

    time.sleep(0.05)


