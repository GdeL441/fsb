import time
import json
import machine
from machine import Pin, PWM, ADC
import _thread
import gc
import network
import socket
from modules import Motors, Sensors, Ultrasonic, Statusled

# Global lock for thread synchronization
lock = _thread.allocate_lock()

# Initialize sensors
# Using pins 26, 27 and 28 (To be changed (NEW PCB))
# 'Sensor.status()' returns True of False based on threshold
L_overline = Sensors.Sensor(28, 12000)
R_overline = Sensors.Sensor(27, 14000)
B_overline = Sensors.Sensor(26, 10000)

# Initialize status sensor (To Be Replaced by RGB Strip)
status_led = Statusled.Statusled(18)

# Initaliaze Motors
Motor_Left = Motors.Motor(14, 15)
Motor_Right = Motors.Motor(17, 16)

# Initialize Ultrasonic, in cm
collision = Ultrasonic.Collision(10, 0, 1)

# Initialize servo motor
servo_pin = PWM(Pin(9))
servo_pin.freq(50)
# Timestamp at which servo was activated, this makes it possible to make the pickup non-blocking
servo_active_time = None

# Is the timer currently running, should the car continue making progres on the steps
started = False

# Keep track if first calibration has been done.
calibrated = False

# Keep track of the time since next_step called, this will help when turning
time_since_next_step = time.time()

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
intersection_detection_time = None

# WiFi configuration
SSID = "Fast Shitbox"
PORT = 80

# PID Constants for line following
Kp = 1.5  # Proportional gain (adjust for faster/slower correction)
Ki = 0.01  # Integral gain (adjust for minor drifting correction)
Kd = 0.2  # Derivative gain (reduces overshoot)

# Base Speed (100% = max = 65535)
BASE_SPEED = 30  # %
TURN_SPEED = 45  # %

# Integral & Derivative Terms for line following
error_sum = 0
last_error = 0

# Add a throttling mechanism for WebSocket messages
last_message_time = time.time()
MIN_MESSAGE_INTERVAL = 0.05  # Minimum time between messages (50ms)

# Thread control flags
server_running = True
motion_running = True
servo_running = True

# Websocket and server variables
websocket = None
message_queue = []
clients = []

# Servo control functions
def set_servo_angle(angle):
    """Set servo angle between 0-180 degrees"""
    # Convert 0-180 degrees to duty cycle (0.5ms to 2.5ms)
    # For 50Hz PWM, period is 20ms, so duty cycle is between 2.5% and 12.5%
    min_duty = 1638  # 0.5ms/20ms * 65535
    max_duty = 8192  # 2.5ms/20ms * 65535
    duty = min_duty + (max_duty - min_duty) * angle / 180
    servo_pin.duty_u16(int(duty))

# WebSocket implementation for MicroPython
class WebSocket:
    def __init__(self, conn):
        self.conn = conn
        self.buffer = b''
        self.closed = False
        
    def send(self, data):
        if self.closed:
            return False
        try:
            # Simple implementation - for production, add proper framing
            self.conn.send(b'\x81' + bytes([len(data)]) + data.encode() if isinstance(data, str) else data)
            return True
        except Exception as e:
            print(f"WebSocket send error: {e}")
            self.closed = True
            return False
            
    def receive(self):
        if self.closed:
            return None
        try:
            data = self.conn.recv(1024)
            if not data:
                self.closed = True
                return None
                
            # Very simplified WebSocket frame parsing
            # In production, implement proper WebSocket protocol
            if data[0] & 0x80:  # FIN bit
                opcode = data[0] & 0x0F
                if opcode == 8:  # Close frame
                    self.closed = True
                    return None
                    
                mask = data[1] & 0x80
                length = data[1] & 0x7F
                
                if length < 126:
                    mask_offset = 2
                elif length == 126:
                    length = (data[2] << 8) | data[3]
                    mask_offset = 4
                else:
                    # 64-bit length not implemented
                    return None
                    
                if mask:
                    mask_key = data[mask_offset:mask_offset+4]
                    data_offset = mask_offset + 4
                    payload = bytearray()
                    for i in range(length):
                        payload.append(data[data_offset + i] ^ mask_key[i % 4])
                    return payload.decode('utf-8')
                else:
                    return data[mask_offset:mask_offset+length].decode('utf-8')
            return None
        except Exception as e:
            print(f"WebSocket receive error: {e}")
            self.closed = True
            return None
            
    def close(self):
        try:
            self.conn.close()
        except:
            pass
        self.closed = True

# Improved WebSocket send function with throttling and error handling
def send_websocket_message(data, important=False):
    """
    Send a message through the WebSocket with throttling and error handling.
    
    Args:
        data: Dictionary to send as JSON
        important: If True, bypass throttling for critical messages
    """
    global websocket, last_message_time, clients
    
    if not clients:
        return False
    
    current_time = time.time()
    
    # Skip non-important messages if we're sending too frequently
    if not important and (current_time - last_message_time < MIN_MESSAGE_INTERVAL):
        return False
    
    try:
        # Convert to JSON only once
        json_data = json.dumps(data)
        
        # Send to all connected clients
        for client in clients[:]:  # Use a copy of the list to allow modification during iteration
            if not client.send(json_data):
                clients.remove(client)
                
        last_message_time = current_time
        return True
    except Exception as e:
        print(f"Error sending WebSocket message: {e}")
        return False

# Reset the cars state, this will be triggered either by the frontend or
# when the car ran into a wall
def reset_state():
    global started, current_step, error_sum, last_error, intersection_detected, intersection_detection_time, robot_pos, robot_heading, green_towers, servo_active_time
    
    with lock:
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
        servo_active_time = None
        set_servo_angle(175)

# Turn the robot to the left with a given speed, used on intersection.
def turn_left(turning_speed=TURN_SPEED):
    print(f"Turn left, speed={turning_speed}")
    Motor_Left.run(-turning_speed)
    Motor_Right.run(turning_speed)

# Turn the robot to the right with a given speed, used on intersection.
def turn_right(turning_speed=TURN_SPEED):
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
    with lock:
        tower = find(
            green_towers, lambda t: t["x"] == robot_pos["x"] and t["y"] == robot_pos["y"]
        )
        if tower != None:
            print("Pick up item with servo arm")
            pickup()
            green_towers.remove(tower)

        current_step += 1
        time_since_next_step = time.time()

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

# Pick up a tower by turning the servo 180 degrees
def pickup():
    global servo_active_time
    with lock:
        set_servo_angle(0)
        servo_active_time = time.time()

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
    if (left < 0) and (right < 0):
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

# Parse WebSocket HTTP headers
def parse_headers(client):
    headers = {}
    while True:
        line = client.readline().decode('utf-8').strip()
        if not line:
            break
        if ':' in line:
            key, value = line.split(':', 1)
            headers[key.strip().lower()] = value.strip()
    return headers

# Handle WebSocket handshake
def handle_websocket_handshake(client, request):
    import hashlib
    import base64
    
    headers = parse_headers(client)
    
    if 'sec-websocket-key' not in headers:
        return False
        
    # Compute the WebSocket accept key
    key = headers['sec-websocket-key']
    magic_string = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
    accept_key = base64.b64encode(hashlib.sha1((key + magic_string).encode()).digest())
    
    # Send the handshake response
    response = b"HTTP/1.1 101 Switching Protocols\r\n"
    response += b"Upgrade: websocket\r\n"
    response += b"Connection: Upgrade\r\n"
    response += b"Sec-WebSocket-Accept: " + accept_key + b"\r\n\r\n"
    
    client.send(response)
    return True

# If there is a connected websocket connection, check if there is a new incoming message
def poll_websocket():
    global started, current_step, error_sum, last_error, MONITORING_SENSOR, Kp, Ki, Kd, BASE_SPEED, TURN_SPEED, steps, robot_pos, robot_heading, MANUAL_CONTROL, MANUAL_CONTROL_SPEEDS, green_towers, clients
    
    if not clients:
        return

    # Check each client for messages
    for client in clients[:]:  # Use a copy to allow modification during iteration
        try:
            data = client.receive()
            if data is None:
                continue
                
            try:
                data = json.loads(data)

                if data["action"] == "start":
                    with lock:
                        time_since_next_step = time.time()
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
                    with lock:
                        started = False
                elif data["action"] == "reset":
                    reset_state()
                elif data["action"] == "monitor_sensor":
                    with lock:
                        MONITORING_SENSOR = not MONITORING_SENSOR
                elif data["action"] == "set_threshold":
                    print("update sensor thresholds", data)
                    L_overline.set_threshold(data["L"])
                    R_overline.set_threshold(data["R"])
                    B_overline.set_threshold(data["B"])
                elif data["action"] == "update_pid":
                    print("update PID parameters", data)
                    with lock:
                        Kp = data["P"]
                        Ki = data["I"]
                        Kd = data["D"]
                elif data["action"] == "update_speed":
                    print("update base and turn speed", data)
                    with lock:
                        speed = max(min(100, data["speed"]), 0)
                        turn_speed = max(min(100, data["turnSpeed"]), 0)
                        TURN_SPEED = turn_speed
                        BASE_SPEED = speed
                elif data["action"] == "manual_control":
                    with lock:
                        started = False
                        MANUAL_CONTROL = not MANUAL_CONTROL
                elif data["action"] == "manual_control_speeds":
                    if not MANUAL_CONTROL:
                        return
                    with lock:
                        MANUAL_CONTROL_SPEEDS = data["speeds"]
                elif data["action"] == "arm_up":
                    if not MANUAL_CONTROL:
                        return
                    with lock:
                        set_servo_angle(0)
                elif data["action"] == "arm_down":
                    if not MANUAL_CONTROL:
                        return
                    with lock:
                        set_servo_angle(175)
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
                client.close()
            except:
                pass
            clients.remove(client)

# Server thread function
def server_thread():
    global server_running, clients
    
    # Initialize WiFi
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=SSID)
    print("My IP address is", ap.ifconfig()[0])
    
    # Initialize server socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', PORT))
    s.listen(5)
    s.setblocking(False)
    
    print("Server started, open for websocket connection")
    
    # Server loop
    while server_running:
        try:
            # Accept new connections
            try:
                client_sock, addr = s.accept()
                print(f"New connection from {addr}")
                client_sock.setblocking(False)
                
                # Read the HTTP request line
                request = client_sock.readline().decode('utf-8').strip()
                if "GET /ws" in request:
                    # Handle WebSocket upgrade
                    if handle_websocket_handshake(client_sock, request):
                        # Add to clients list
                        ws = WebSocket(client_sock)
                        with lock:
                            clients.append(ws)
                        status_led.connected()
                else:
                    # Not a WebSocket request, close connection
                    client_sock.close()
            except OSError as e:
                # No new connections
                pass
                
            # Process existing clients
            if clients:
                # If there is a websocket connection, send all queued messages
                with lock:
                    for client in clients[:]:
                        if client.closed:
                            clients.remove(client)
                            continue
                            
                    while len(message_queue) > 0 and clients:
                        msg = message_queue.pop(0)
                        for client in clients[:]:
                            if not client.send(msg):
                                clients.remove(client)
                
                poll_websocket()
            else:
                status_led.loading_animation()
        except Exception as e:
            print(f"Server error: {e}")
        
        time.sleep(0.01)
        gc.collect()  # Garbage collection to free memory

# Motion control thread function
def motion_thread():
    global motion_running, started, current_step, error_sum, last_error, intersection_detected, calibrated
    
    # Initial calibration
    if not calibrated:
        calibrate_all(0.9, False)
        with lock:
            calibrated = True
    
    # Motion control loop
    while motion_running:
        with lock:
            is_started = started
            is_manual = MANUAL_CONTROL
            is_monitoring = MONITORING_SENSOR
            current_step_value = current_step
            steps_value = steps.copy() if len(steps) > 0 else []
        
        if is_started:
            # Check for collision
            if collision.detect():
                print("Collision Detected! Resetting...")
                status_led.collision()
                reset_state()
                continue
            
            # Get current step
            if current_step_value < len(steps_value):
                current_action = steps_value[current_step_value]
            else:
                current_action = None
            
            if current_action == "FORWARD":
                # Line following logic
                if L_overline.status() and R_overline.status():
                    # Both sensors on line -> intersection detected
                    with lock:
                        intersection_detected = True
                        intersection_detection_time = time.time()
                
                with lock:
                    is_intersection = intersection_detected
                    time_since_step = time.time() - time_since_next_step
                
                if B_overline.status() and is_intersection and time_since_step > 0.9:
                    # At intersection, move to next step
                    possible_next_step = get_next_step()
                    if possible_next_step == None or possible_next_step != "FORWARD":
                        stop_motors()
                    
                    with lock:
                        intersection_detected = False
                    
                    next_step()
                else:
                    # PID line following
                    error = (L_overline.value() - R_overline.value()) / 65535.0
                    
                    with lock:
                        # Update PID variables
                        error_sum += error
                        # Anti-windup: Limit integral term
                        MAX_INTEGRAL = 1.0
                        error_sum = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, error_sum))
                        
                        # Compute correction
                        error_derivative = error - last_error
                        last_error = error
                        correction = (Kp * error) + (Ki * error_sum) + (Kd * error_derivative)
                        
                        # Get current speed settings
                        base_speed = BASE_SPEED
                    
                    # Adjust motor speeds
                    left_speed = int(base_speed + (correction * base_speed))
                    right_speed = int(base_speed - (correction * base_speed))
                    
                    # Apply speed limits
                    left_speed = max(-100, min(100, left_speed))
                    right_speed = max(-100, min(100, right_speed))
                    
                    # Apply speeds to motors
                    Motor_Left.run(left_speed)
                    Motor_Right.run(right_speed)
            
            elif current_action == "RIGHT":
                # Turn right
                turn_right()
                
                # Check if turn is complete
                if (R_overline.status() or L_overline.status()) and time.time() - time_since_next_step > 0.5:
                    stop_motors()
                    next_step()
            
            elif current_action == "LEFT":
                # Turn left
                turn_left()
                
                # Check if turn is complete
                if (R_overline.status() or L_overline.status()) and time.time() - time_since_next_step > 0.5:
                    stop_motors()
                    next_step()
        
        elif is_manual:
            # Manual control mode
            manual_control()
            status_led.manual_control()
        
        elif is_monitoring:
            # Monitoring mode
            send_sensor_values()
            status_led.waiting_for_orders()
        
        else:
            # Idle mode
            stop_motors()
            status_led.waiting_for_orders()
        
        time.sleep(0.01)  # Small delay to prevent CPU hogging

# Servo control thread function
def servo_thread():
    global servo_running, servo_active_time
    
    while servo_running:
        with lock:
            active_time = servo_active_time
        
        if active_time is not None:
            if time.time() - active_time > 0.7:
                with lock:
                    set_servo_angle(175)
                    servo_active_time = None
        
        time.sleep(0.05)  # Servo doesn't need frequent updates

# Main function to start all threads
def main():
    # Start server thread
    _thread.start_new_thread(server_thread, ())
    
    # Start motion control thread
    _thread.start_new_thread(motion_thread, ())
    
    # Start servo control thread
    _thread.start_new_thread(servo_thread, ())
    
    # Main thread becomes a watchdog/manager
    try:
        while True:
            # Monitor memory usage
            free_mem = gc.mem_free()
            if free_mem < 10000:  # Arbitrary threshold, adjust as needed
                print(f"Low memory warning: {free_mem} bytes free")
                gc.collect()
            
            # Could add more management tasks here
            
            time.sleep(1)  # Check every second
    
    except KeyboardInterrupt:
        # Clean shutdown
        global server_running, motion_running, servo_running
        print("Shutting down threads...")
        server_running = False
        motion_running = False
        servo_running = False
        stop_motors()
        time.sleep(1)  # Give threads time to exit
        print("Shutdown complete")

# Start the program
if __name__ == "__main__":
    print("Starting multithreaded robot control")
    main()
