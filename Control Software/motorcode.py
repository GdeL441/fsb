import time
import pwmio
import ipaddress
import board
import digitalio
from analogio import AnalogIn
import socketpool
import wifi
from adafruit_httpserver import Server, Request, Response, GET, Websocket
import mdns
import json

# Initialize sensors
# Using GP26, 27 and 28

# Voor de motors: 1 pin met PWM signaal en een andere met ON/OFF voor direction
# Define motor driver pins
MOTOR_SPEED = pwmio.PWMOut(board.GP2, frequency=1000)  # Motor speed
MOTOR_DIRECTION = digitalio.DigitalInOut(board.GP3)  # Motor direction
MOTOR_DIRECTION.direction = digitalio.Direction.OUTPUT # Set the direction as an output pin


# WiFi configuration
SSID = "PICO-FSB-502"
# PASSWORD = "password"  #Verander voor veiligheidsredenen, wat is veiligheid?
PORT = 80

# Initialize mDNS
mdns_server = mdns.Server(wifi.radio)
mdns_server.hostname = "fast-shitbox"
mdns_server.advertise_service(service_type="_http", protocol="_tcp", port=PORT)
print("mDNS advertised: _http._tcp, hostname='fast-shitbox'")

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
        websocket.send_message(data, fail_silently=True)
        data = json.loads(data)
        #This code starts the motor
        if data["action"] == "start_motor":
            motors_run(int(data["speed"]), data["direction"])


server.start(port=PORT)
print("Server started, open for websocket connection")


def motors_run(speed, direction):
    duty_cycle = int(speed * 65535 / 100)
    MOTOR_SPEED.duty_cycle = duty_cycle
    if direction == "forward":
        MOTOR_DIRECTION.value = True
    else: 
        MOTOR_DIRECTION.value = False
    print(MOTOR_DIRECTION.value)
    print(duty_cycle)


# Main loop
while True:
    server.poll()

    if websocket is not None:
        poll_websocket()

    time.sleep(0.05)
