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
from modules import Motors, Sensors


# Initialize sensors
# Using GP26, 27 and 28
# Returns True of False
L_overline = Sensors.Sensor(board.GP26, 30000)
R_overline = Sensors.Sensor(board.GP27, 40000)
B_overline = Sensors.Sensor(board.GP28, 30000)

# Initialize status sensor (To Be Replaced by RGB Strip)
status_led = digitalio.DigitalInOut(board.LED)
status_led.direction = digitalio.Direction.OUTPUT

# Initaliaze Motors
Motor_Left = Motors.Motor(board.GP2, board.GP3)
Motor_Right = Motors.Motor(board.GP4, board.GP5)


started = False
# Keep track of the time since next_step called, this will help when turning
time_since_next_step = time.monotonic()

# Initialize robot position/heading and grid
robot_pos = {"x": 1, "y": 1}
robot_heading = "N"  # "N" "E" "S" "W"

# Steps the robot should take, later this should be computed at runtime(grid backtracking)
# steps = ["FORWARD", "LEFT", "FORWARD", "RIGHT", "FORWARD"]
steps = ["FORWARD", "FORWARD", "LEFT"]
current_step = 0
intersection_detected = False

# WiFi configuration
SSID = "Fast Shitbox"
# PASSWORD = "password"  #Verander voor veiligheidsredenen, wat is veiligheid?
PORT = 80

wifi.radio.start_ap(ssid=SSID)

pool = socketpool.SocketPool(wifi.radio)
server = Server(pool, "/static", debug=True)
websocket = None

# print IP adres
print("My IP address is", wifi.radio.ipv4_address_ap)




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
    global started, websocket
    assert websocket != None

    data = websocket.receive(fail_silently=True)
    if data is not None:
        data = json.loads(data)

        if data["action"] == "start":
            started = True
        elif data["action"] == "stop":
            started = False
        elif data["action"] == "move":
            print(data["speedL"], data["speedR"])
        else:
            print("Received other data: ", data)
        


server.start(port=PORT)
print("Server started, open for websocket connection")


def over_line(sensor):
    return sensor["pin"].value < sensor["threshold"]

def move_forward(speed = 50):
    Motor_Left.run(speed)
    Motor_Right.run(speed)
    print("Forward")

def turn_left(right_speed = 50):
    Motor_Left.run(-right_speed)
    Motor_Right.run(right_speed)
    print("Turn left")

def nudge_left(nudge_speed = 30):
    Motor_Left.run(50 - nudge_speed)
    Motor_Right.run(50 + nudge_speed)
    print("Nudge Left")

def turn_right(left_speed = 50):
    Motor_Right.run(-left_speed)
    Motor_Left.run(left_speed)
    print("Turn right")

def nudge_right(nudge_speed = 30):
    Motor_Left.run(50 + nudge_speed)
    Motor_Right.run(50 - nudge_speed)
    print("Nudge Right")

def stop_motors():
    Motor_Right.stop()
    Motor_Left.stop()
    print("Stop")


def next_step():
    global current_step, started, time_since_next_step, intersection_detected
    current_step += 1
    time_since_next_step = time.monotonic()
    if current_step + 1 == len(steps):
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


# Main loop
while True:
    print(f"Sensor left: {L_overline.status()}")
    print(f"Sensor right: {R_overline.status()}")
    print(f"Sensor back: {B_overline.status()}")

    if started == True:
        if steps[current_step] == "FORWARD":
            # If the current step is moving forward, just follow the line until the next intersections

            if not L_overline.status() and not R_overline.status():
                # TODO: is if statement necessary?
                move_forward()
            elif L_overline.status() and R_overline.status():
                # Both sensors on line -> intersection detected
                intersection_detected = True
                print("Intersection Detected")
                move_forward()
            elif L_overline.status():
                # Left sensors on line -> robot should correct by steering left
                nudge_left()
            elif R_overline.status():
                # Right sensors on line -> robot should correct by steering right
                nudge_right()
            else: 
                print("lost")
                turn_left()

            if B_overline.status() and intersection_detected:
                # A intersections was detected and now we are at the intersection -> move to next step
                intersection_detected = False
                stop_motors()
                next_step()

        else:
            # if time_since_next_step < 0.5: 
            #     print("still turning...")
            # else: 
            # The robot is currently turning, wait until back on line before moving to next step
            # TODO: add reference time so this doesn't trigger before the turn even started
            if steps[current_step] == "RIGHT":
                # If the robot is turning right, it should stop turning once the left sensor hits the black line
                turn_right()
                if L_overline.status() and time.monotonic() - time_since_next_step > 0.5:
                    stop_motors()
                    next_step()
            elif steps[current_step] == "LEFT" :
                # If the robot is turning left, it should stop turning once the right sensor hits the black line
                turn_left()
                if R_overline.status() and time.monotonic() - time_since_next_step > 0.5:
                    stop_motors()
                    next_step()
    else:
        stop_motors()

    server.poll()

    if websocket is not None:
        poll_websocket()

    time.sleep(0.05)
