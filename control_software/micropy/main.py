import time
import json
import machine
from machine import Pin, PWM, ADC
import network
from modules import Motors, Sensors, Ultrasonic, Statusled
from microdot import Microdot, Response, send_file
from microdot.websocket import with_websocket
import _thread
import asyncio

print("test")

# Initialize sensors
# Using GP26, 27 and 28 (To be changed (NEW PCB))
# 'Sensor.status()' returns True of False based on threshold
L_overline = Sensors.Sensor(Pin(28), 12000)
R_overline = Sensors.Sensor(Pin(27), 14000)
B_overline = Sensors.Sensor(Pin(26), 10000)

# Calibration threshold
L_R_calibration_threshold = 0.8
B_calibration_threshold = 0.9

# Initialize status sensor (To Be Replaced by RGB Strip)
status_led = Statusled.Statusled(Pin(18))

# Initialize Motors
Motor_Left = Motors.Motor(Pin(14), Pin(15))
Motor_Right = Motors.Motor(Pin(17), Pin(16))

# Initialize Ultrasonic, in cm
# collision = Ultrasonic.Ultrasonic(machine.UART(0), 15)

# Initialize servo motor
servo_pwm = PWM(Pin(9))
servo_pwm.freq(50)
servo_min = 500
servo_max = 2500

def set_servo_angle(angle):
    duty = int((angle / 180) * (servo_max - servo_min) + servo_min)
    servo_pwm.duty_ns(duty * 1000)

# Timestamp at which servo was activated
servo_active_time = None
timeout_time = None

# Is the timer currently running
started = False

# Add servo constants:
ARM_DOWN = 175
ARM_UP = 10

# Keep track if first calibration has been done.
calibrated = False

# Has the car finished? (For the statusled)
finished = False

# Keep track of the time since next_step called
time_since_next_step = time.ticks_ms()

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

# Steps the robot should take
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
intersection_detected = False

# PID Constants for line following
Kp = 1.5  # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.2  # Derivative gain


# Base Speed (100% = max = 1023 for PWM)
BASE_SPEED = 30  # %
TURN_SPEED = 30  # %

# Integral & Derivative Terms
error_sum = 0
last_error = 0
turn_error_sum = 0
turn_last_error = 0

# WiFi configuration
SSID = "Fast Shitbox"
PASSWORD = "password"
PORT = 80

print("test")
# Setup WiFi AP
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=SSID, password=PASSWORD)
print("Access point started")
print("Network config:", ap.ifconfig())

print("test")
# WebSocket state
websocket = None
last_message_time = time.ticks_ms()
MIN_MESSAGE_INTERVAL = 50  # ms

# Create Microdot app
app = Microdot()
Response.default_content_type = 'text/html'

# Serve static files
@app.route('/')
def index(request):
    return send_file('static/index.html')

@app.route('/static/<path:path>')
def static(request, path):
    return send_file('static/' + path)

# Add an error handler for the Microdot app
@app.errorhandler(Exception)
def handle_exception(request, exception):
    print(f"Server error: {type(exception).__name__}: {exception}")
    return {"error": str(exception)}, 500
# WebSocket route
@app.route('/ws')
@with_websocket
async def ws(request, ws):
    global websocket
    
    # Check if there's already an active connection and close it
    if websocket is not None:
        try:
            await ws.close()
        except:
            pass
    
    # Store the websocket connection
    websocket = ws
    status_led.connected()
    print("WebSocket client connected")
    
    # Send initial setup data
    await send_setup_data()
    
    try:
        # Handle incoming messages
        while True:
            data = await ws.receive()
            if data:
                await handle_websocket_message(data)
    except Exception as e:
        print(f"WebSocket error: {type(e).__name__}: {e}")
    finally:
        # Make sure to properly clean up the websocket variable when connection ends
        if websocket == ws:  # Only clear if it's still the same connection
            websocket = None
            print("WebSocket client disconnected")

async def send_setup_data():
    """Send initial setup data to the client"""
    if websocket is None:
        pass
        
    data = {
        "action": "setup",
        "L": L_overline.threshold,
        "R": R_overline.threshold,
        "B": B_overline.threshold,
        "L_R_calibration_threshold": L_R_calibration_threshold,
        "B_calibration_threshold": B_calibration_threshold,
        "speed": BASE_SPEED,
        "turnSpeed": TURN_SPEED,
        "P": Kp,
        "I": Ki,
        "D": Kd
    }
    await send_websocket_message(data, important=True)
    print(f"Sent setup data: {data}")

async def send_websocket_message(data, important=False):
    global websocket, last_message_time
    
    if websocket is None:
        return False
    
    current_time = time.ticks_ms()
    
    if not important and (current_time - last_message_time < MIN_MESSAGE_INTERVAL):
        return False
    
    try:
        json_data = json.dumps(data)
        await websocket.send(json_data)
        last_message_time = current_time
        return True
    except Exception as e:
        print("Error sending message:", e)
        return False

async def handle_websocket_message(message):
    global started, current_step, error_sum, last_error
    global MONITORING_SENSOR, Kp, Ki, Kd, BASE_SPEED, TURN_SPEED
    global steps, robot_pos, robot_heading, MANUAL_CONTROL, MANUAL_CONTROL_SPEEDS
    global green_towers, time_since_next_step, L_R_calibration_threshold, B_calibration_threshold
    
    try:
        data = json.loads(message)
        print("Received:", data)
        
        if data["action"] == "get_setup":
            await send_setup_data()
        elif data["action"] == "start":
            time_since_next_step = time.ticks_ms()
            MANUAL_CONTROL = False
            if all(key in data for key in ["path", "startX", "startY", "heading", "green_towers"]):
                steps = data["path"]
                robot_pos = {"x": data["startX"], "y": data["startY"]}
                robot_heading = data["heading"]
                green_towers = data["green_towers"]

            if "thresholds" in data:
                L_overline.set_threshold(data["thresholds"]["L"])
                R_overline.set_threshold(data["thresholds"]["R"])
                B_overline.set_threshold(data["thresholds"]["B"])

            if "speed" in data:
                BASE_SPEED = max(min(100, data["speed"]), 0)
            if "turnSpeed" in data:
                TURN_SPEED = max(min(100, data["turnSpeed"]), 0)

            started = True
            if len(steps) > 0:
                if steps[0] == "LEFT":
                    turn_left()
                elif steps[0] == "RIGHT":
                    turn_right()

        elif data["action"] == "stop":
            started = False
        elif data["action"] == "reset":
            reset_state()
        elif data["action"] == "monitor_sensor":
            MONITORING_SENSOR = not MONITORING_SENSOR
        elif data["action"] == "set_threshold":
            L_overline.set_threshold(data["L"])
            R_overline.set_threshold(data["R"])
            B_overline.set_threshold(data["B"])
            L_R_calibration_threshold = data["L_R_calibration_threshold"]
            B_calibration_threshold = data["B_calibration_threshold"]
        elif data["action"] == "update_pid":
            Kp = data["P"]
            Ki = data["I"]
            Kd = data["D"]
        elif data["action"] == "update_speed":
            BASE_SPEED = max(min(100, data["speed"]), 0)
            TURN_SPEED = max(min(100, data.get("turnSpeed", TURN_SPEED)), 0)
        elif data["action"] == "disable_manual_control":
            started = False
            MANUAL_CONTROL = False
        elif data["action"] == "enable_manual_control":
            started = False
            MANUAL_CONTROL = True
        elif data["action"] == "manual_control_speeds":
            if MANUAL_CONTROL:
                MANUAL_CONTROL_SPEEDS = data["speeds"]
        elif data["action"] == "arm_up":
            if MANUAL_CONTROL:
                set_servo_angle(ARM_UP)
        elif data["action"] == "arm_down":
            if MANUAL_CONTROL:
                set_servo_angle(ARM_DOWN)
        elif data["action"] == "calibrate":
            await calibrate_all(True)
    except Exception as e:
        print("Error handling WebSocket message:", e)

def reset_state():
    global started, current_step, error_sum, last_error, intersection_detected
    global robot_pos, robot_heading, green_towers, servo_active_time, finished
    
    started = False
    stop_motors()
    current_step = 0
    error_sum = 0
    last_error = 0
    intersection_detected = False
    robot_pos = {"x": 6, "y": 0}
    robot_heading = "N"
    green_towers = []
    servo_active_time = None
    set_servo_angle(ARM_DOWN)
    finished = False

def turn_left():
    global time_since_next_step, last_error, error_sum
    last_error = 0
    error_sum = 0
    
    turn_elapsed_time = time.ticks_diff(time.ticks_ms(), time_since_next_step) / 1000
    
    if turn_elapsed_time < 0.5:
        turn_speed = TURN_SPEED
    else:
        turn_speed = TURN_SPEED * 0.7
    
    Motor_Left.run(-turn_speed)
    Motor_Right.run(turn_speed)

def turn_right():
    global time_since_next_step, last_error, error_sum
    last_error = 0
    error_sum = 0
    
    turn_elapsed_time = time.ticks_diff(time.ticks_ms(), time_since_next_step) / 1000
    
    if turn_elapsed_time < 0.5:
        turn_speed = TURN_SPEED
    else:
        turn_speed = TURN_SPEED * 0.7
    
    Motor_Left.run(turn_speed)
    Motor_Right.run(-turn_speed)

def stop_motors():
    Motor_Right.stop()
    Motor_Left.stop()

async def update_pos_and_heading(send=True):
    global robot_pos, robot_heading

    if steps[current_step] == "FORWARD":
        if robot_heading == "N":
            robot_pos["y"] += 1
        elif robot_heading == "S":
            robot_pos["y"] -= 1
        elif robot_heading == "E":
            robot_pos["x"] += 1
        elif robot_heading == "W":
            robot_pos["x"] -= 1
    elif steps[current_step] == "RIGHT":
        robot_heading = DIRECTIONS[(DIRECTIONS.index(robot_heading) + 1) % 4]
    elif steps[current_step] == "LEFT":
        robot_heading = DIRECTIONS[(DIRECTIONS.index(robot_heading) - 1) % 4]

    if send:
        data = {
            "action": "position_updated",
            "position": robot_pos,
            "heading": robot_heading,
        }
        await send_websocket_message(data, important=True)

def get_next_step():
    if current_step + 1 == len(steps):
        return None
    return steps[current_step + 1]

async def maybe_pickup():
    global robot_pos, robot_heading
    current_pos = robot_pos.copy()
    current_heading = robot_heading

    await update_pos_and_heading(False)

    tower = next((t for t in green_towers if t["x"] == robot_pos["x"] and t["y"] == robot_pos["y"]), None)
    if tower is not None:
        await pickup()
        green_towers.remove(tower)

    robot_pos = current_pos
    robot_heading = current_heading

def should_pickup_next_step():
    global robot_pos, robot_heading, current_step, steps, green_towers, servo_active_time
    
    if servo_active_time is None:
        return False
    
    if current_step + 1 >= len(steps) or steps[current_step + 1] != "FORWARD":
        return False
    
    # Calculate position after current step
    if steps[current_step] == "FORWARD":
        if robot_heading == "N":
            next_y = robot_pos["y"] + 1
            next_x = robot_pos["x"]
            next_heading = robot_heading
        elif robot_heading == "S":
            next_y = robot_pos["y"] - 1
            next_x = robot_pos["x"]
            next_heading = robot_heading
        elif robot_heading == "E":
            next_y = robot_pos["y"]
            next_x = robot_pos["x"] + 1
            next_heading = robot_heading
        elif robot_heading == "W":
            next_y = robot_pos["y"]
            next_x = robot_pos["x"] - 1
            next_heading = robot_heading
    elif steps[current_step] == "RIGHT":
        next_heading = DIRECTIONS[(DIRECTIONS.index(robot_heading) + 1) % 4]
        next_x = robot_pos["x"]
        next_y = robot_pos["y"]
    elif steps[current_step] == "LEFT":
        next_heading = DIRECTIONS[(DIRECTIONS.index(robot_heading) - 1) % 4]
        next_x = robot_pos["x"]
        next_y = robot_pos["y"]
    else:
        return False
    
    # Calculate position after next step
    if next_heading == "N":
        final_y = next_y + 1
        final_x = next_x
    elif next_heading == "S":
        final_y = next_y - 1
        final_x = next_x
    elif next_heading == "E":
        final_y = next_y
        final_x = next_x + 1
    elif next_heading == "W":
        final_y = next_y
        final_x = next_x - 1
    
    return any(t["x"] == final_x and t["y"] == final_y for t in green_towers)

async def next_step():
    global current_step, started, time_since_next_step, intersection_detected, finished

    await update_pos_and_heading()

    current_step += 1
    time_since_next_step = time.ticks_ms()

    if current_step == len(steps):
        reset_state()
        data = {"action": "finished"}
        finished = True
        await send_websocket_message(data, important=True)
        return
    else:
        if steps[current_step] == "LEFT":
            turn_left()
        elif steps[current_step] == "RIGHT":
            turn_right()

    data = {"action": "next_step", "step": steps[current_step]}
    await send_websocket_message(data, important=True)

async def pickup():
    global servo_active_time, timeout_time
    timeout_time = time.ticks_ms()
    set_servo_angle(ARM_UP)
    servo_active_time = time.ticks_ms()
    
    data = {
        "action": "tower_collected",
        "position": robot_pos,
        "remaining_towers": len(green_towers)
    }
    await send_websocket_message(data, important=True)

async def send_sensor_values():
    data = {
        "action": "sensor_values",
        "L": L_overline.value(),
        "R": R_overline.value(),
        "B": B_overline.value(),
    }
    await send_websocket_message(data)

def manual_control():
    left, right = MANUAL_CONTROL_SPEEDS["left"], MANUAL_CONTROL_SPEEDS["right"]
    Motor_Left.run(left)
    Motor_Right.run(right)
    if left < 0 and right < 0:
        status_led.reverse()

async def calibrate_all(send=True):
    status_led.calibration()
    r_threshold = R_overline.calibrate(L_R_calibration_threshold)
    l_threshold = L_overline.calibrate(L_R_calibration_threshold)
    b_threshold = B_overline.calibrate(B_calibration_threshold)
    status_led.calibration_finished()
    
    if send:
        data = {
            "action": "thresholds_updated",
            "thresholds": {
                "L": l_threshold,
                "R": r_threshold,
                "B": b_threshold,
                "L_R_calibration_threshold": L_R_calibration_threshold,
                "B_calibration_threshold": B_calibration_threshold
            }
        }
        await send_websocket_message(data, important=True)

def get_intersection_delay():
    return 0.1 + (1.0 - (BASE_SPEED / 100.0)) * 0.4

def get_turning_delay():
    return 0.1 + (1.0 - (TURN_SPEED / 100.0)) * 0.4

async def check_for_intersection():
    global intersection_detected
    
    if L_overline.status() and R_overline.status() and not intersection_detected:
        intersection_detected = True
        await maybe_pickup()
        
# Start the Microdot server in a background task
async def start_server():
    try:
        await app.start_server(host='0.0.0.0', port=PORT, debug=True)
    except Exception as e:
        print("Server error:", e)
        machine.reset()  # Reset the device if the server crashes


async def run_main_loop():
    global servo_active_time, timeout_time, started, current_step, error_sum, last_error, intersection_detected, robot_pos, robot_heading, green_towers, steps, time_since_next_step, MONITORING_SENSOR, MANUAL_CONTROL, MANUAL_CONTROL_SPEEDS, websocket, last_message_time, poll_counter, led_counter, websocket_counter, possible_next_step, pickup_next_step, error, error_derivative, left_speed, right_speed, msg, data, left_status, right_status, back_status, intersection_detection_time
    global finished
    # Main control loop
    while True:
        print("Main loop")
        # if not calibrated:
        #     calibrate_all(False)
        #     calibrated = True

        if started:
            # if collision.detect():
            #     print("Collision Detected! Resetting...")
            #     status_led.collision()
            #     reset_state()

            print("negawatt")
            await check_for_intersection()

            if timeout_time and time.ticks_diff(time.ticks_ms(), timeout_time) < 500:
                continue
            else:
                timeout_time = None

            if steps[current_step] == "FORWARD":
                if ((B_overline.status() and intersection_detected) and 
                    time.ticks_diff(time.ticks_ms(), time_since_next_step) > 500):
                    
                    possible_next_step = get_next_step()
                    if possible_next_step is None or possible_next_step != "FORWARD":
                        stop_motors()

                    if possible_next_step == "FORWARD" and should_pickup_next_step():
                        stop_motors()
                        set_servo_angle(ARM_DOWN)

                    intersection_detected = False
                    await next_step()
                else:
                    error = (L_overline.value() - R_overline.value()) / 65535.0
                    error_sum += error
                    error_derivative = error - last_error
                    last_error = error

                    MAX_INTEGRAL = 1.0
                    error_sum = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, error_sum))

                    correction = (Kp * error) + (Ki * error_sum) + (Kd * error_derivative)

                    left_speed = int(BASE_SPEED + (correction * BASE_SPEED))
                    right_speed = int(BASE_SPEED - (correction * BASE_SPEED))

                    if intersection_detected:
                        Motor_Left.run(left_speed * 0.7)
                        Motor_Right.run(right_speed * 0.7)
                    else:
                        Motor_Left.run(left_speed)
                        Motor_Right.run(right_speed)
            else:
                if steps[current_step] == "RIGHT":
                    turn_right()
                    if ( (R_overline.status() or L_overline.status() ) and  
                        ( time.ticks_diff(time.ticks_ms(), time_since_next_step) > get_turning_delay() * 1000) ):
                        stop_motors()
                        await next_step()
                elif steps[current_step] == "LEFT":
                    turn_left()
                    if ( (L_overline.status() or R_overline.status() ) and 
                        time.ticks_diff(time.ticks_ms(), time_since_next_step) > get_turning_delay() * 1000):
                        stop_motors()
                        await next_step()
        else:
            if MANUAL_CONTROL:
                manual_control()
                status_led.manual_control()
            elif MONITORING_SENSOR:
                await send_sensor_values()
            else:
                stop_motors()
                if websocket is not None and not finished:
                    status_led.waiting_for_orders()
                elif websocket is not None and finished:
                    status_led.finished()

        if servo_active_time is not None:
            status_led.collection()
            if time.ticks_diff(time.ticks_ms(), servo_active_time) > 700:
                servo_active_time = None
                set_servo_angle(ARM_DOWN)
        elif started:
            status_led.next_object()
        elif finished:
            status_led.finished()
        elif websocket is None:
            status_led.loading_animation()

        # Small delay to prevent CPU hogging
        # time.sleep(0.01)

def run_async_server():
    asyncio.run(start_server())

if __name__ == "__main__":
    _thread.start_new_thread(run_main_loop, ())
    run_async_server()
