import time
import board
import digitalio
from analogio import AnalogIn
import socketpool
import wifi
from adafruit_httpserver import Server, Request, Response, GET, Websocket
import mdns

# Initialize sensors 
# Using GP26, 27 and 28
sensor1 = {
    "pin": AnalogIn(board.GP26),
    "threshold" : 25000
}
sensor1 = {
    "pin": AnalogIn(board.GP27),
    "threshold" : 25000
}
sensor1 = {
    "pin": AnalogIn(board.GP28),
    "threshold" : 25000
}

def over_line(sensor):
    return sensor["pin"].value < sensor["threshold"]


while True:
    print(f"Sensor 1: {over_line(sensor1)}")
    #print(f"Sensor 2: {get_intensity(sensor2)}")
    #print(f"Sensor 3: {get_intensity(sensor3)}")
    time.sleep(0.1)

'''

SSID = "PICO FSB"  #Verander X naar groepsnummer
#PASSWORD = "password"  #Verander voor veiligheidsredenen
wifi.radio.start_ap(ssid=SSID)
# Initialize mDNS
mdns_server = mdns.Server(wifi.radio)
mdns_server.hostname = "fast-shitbox"
mdns_server.advertise_service(service_type="_http", protocol="_tcp", port=4000)

# print IP adres
print("My IP address is", wifi.radio.ipv4_address_ap)

pool = socketpool.SocketPool(wifi.radio)
server = Server(pool, "/static", debug=True)
websocket = None

# Deze functie wordt uitgevoerd wanneer de server een HTTP request ontvangt
@server.route("/connect-websocket", GET)
def connect_client(request: Request):
    global websocket  # pylint: disable=global-statement

    if websocket is not None:
        websocket.close()  # Close any existing connection

    websocket = Websocket(request)

    return websocket

server.start(str(wifi.radio.ipv4_address_ap))
print("Server started")
while True:
    server.poll()

    if websocket is not None:
        data = websocket.receive(fail_silently=True)
        if data is not None:
            print(data)
            websocket.send_message(data, fail_silently=True)
    time.sleep(0.1)
'''