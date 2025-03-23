
#define VELOCITY_TEMP(temp)       ( ( 331.5 + 0.6 * (float)( temp ) ) * 100 / 1000000.0 ) // The ultrasonic velocity (cm/us) compensated by temperature
#define POWER_PIN  25 // ESP32 pin GPIO25 connected to sensor's VCC pin
#define trigechoPin 26 // ESP32 pin GPIO26 connected to sensor's D pin

float distance;
uint32_t pulseWidthUs;

class MyCustomSensor : public PollingComponent, public Sensor {
 public:
  MyCustomSensor() : PollingComponent(1800000ull) { }
  Sensor *distance_sensor = new Sensor();
  Sensor *percentage_sensor = new Sensor();


  void setup() override {
   //empty
  }
  void update() override {
    int16_t  dist, temp;
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH); //Turns the sensor on
    delay(50);
    
    pinMode(trigechoPin,OUTPUT);
    digitalWrite(trigechoPin,LOW);
    
    digitalWrite(trigechoPin,HIGH); //Set the trig pin High
    delayMicroseconds(10);     //Delay of 10 microseconds
    digitalWrite(trigechoPin,LOW); //Set the trig pin Low

    pinMode(trigechoPin,INPUT);  //Set the pin to input mode
    pulseWidthUs = pulseIn(trigechoPin,HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)

    distance = pulseWidthUs * VELOCITY_TEMP(10) / 2.0;  //The distance can be calculated according to the flight time of ultrasonic wave,
                                                        //and the ultrasonic sound speed can be compensated according to the actual ambient temperature
    distance_sensor->publish_state(distance); //Transmit distance to water level in cm

    percentage_sensor->publish_state((175 - distance) / 1.45);  // Transmit water level in percentage 
                                                                // (maximum level at 30cm below sensor, minimum at 175cm below sensor)

    digitalWrite(POWER_PIN, LOW); //turn sensor off
  }
};