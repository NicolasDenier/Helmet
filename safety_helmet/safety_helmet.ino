/*  Safety helmet code
        11/2021
    Send GPS coordinates via LoRa WAN
    Detect fall with accelerometer and send LoRa message
    Turn on lights when it's dark (and off when it's not)
*/

// Libraries
#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Pins
#define RESET 15
#define RX_GPS 6
#define TX_GPS 5
#define RX_LORA 4
#define TX_LORA 3
#define LEDs 7
#define LightSensor A3

// Baudrates
static const uint32_t SerialMonitorBaud = 115200;
static const uint32_t GPSBaud = 9600;
static const uint32_t LoraBaud = 57600;

// Global variables
char latitude[15];
char longitude[15];
char message[35];
int brightness;
float g; // gravity acceleration
bool fall = false; // true if a fall is detected

// Accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// The TinyGPS++ object
TinyGPSPlus gps;

// Serial connections (be careful to use only one at a time)
SoftwareSerial GPSSerial(RX_GPS, TX_GPS); // Serial connection to the GPS device
SoftwareSerial LoraSerial(RX_LORA, TX_LORA); // Serial connection to the Lora device

// Create an instance of the rn2xx3 library, giving the software UART as stream to use, and using LoRa WAN
rn2xx3 myLora(LoraSerial);



// -------------------- SETUP --------------------
void setup() {

  // Init inputs/outputs
  pinMode(LEDs, OUTPUT);
  pinMode(LightSensor, INPUT);
  //pinMode(LED_BUILTIN, OUTPUT); // Builtin LED

  // Open serial communications and wait for port to open:
  LoraSerial.begin(LoraBaud);
  Serial.begin(SerialMonitorBaud);
  delay(1000); // Wait for the arduino ide's serial console to open


  // Check accelerometer
  if (!accel.begin())
  {
    Serial.println("Accelerometer not found");
  }

  Serial.println("Startup");

  initialize_radio();

  //transmit a startup message
  //myLora.tx("TTN Mapper on Uno node");
  LoraSerial.end();

  delay(3000);
}


// -------------------- LOOP --------------------
void loop() {

  // Get photoresistor value
  brightness = analogRead(LightSensor);

  // Backlight on if it's dark:
  Serial.print("Brightness: "); Serial.println(brightness);
  if (brightness <= 150 && brightness >= 30) // May need to change the values
  {
    digitalWrite(LEDs, HIGH);
    Serial.println("Backlight ON");
  } else {
    digitalWrite(LEDs, LOW);
    Serial.println("Backlight OFF");
  }


  // Get accelerometer values
  sensors_event_t event;
  accel.getEvent(&event);

  // Compute g value and compare it to a treshold:
  g = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
  if (g > 20) {
    Serial.println("Fall");
    fall = true;
  }


  // GPS
  GPSSerial.begin(GPSBaud);
  delay(1000);

  while (GPSSerial.available() > 0)
    if (gps.encode(GPSSerial.read()))
      getGPSInfo();

  // Check GPS module
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }

  GPSSerial.end();
  delay(1000);


  // LoRa
  LoraSerial.begin(LoraBaud);
  delay(1000);
  myLora.sendRawCommand(F("mac set adr on"));
  Serial.println("OK");
  delay(500);

  // Prepare message:
  strcpy(message, latitude);
  strcat(message, ",");
  strcat(message, longitude);
  //Serial.print("message: ");
  //Serial.println(message);

  // Send message with LoRa
  myLora.tx(message);

  // Send fall if falling
  if (fall) {
    myLora.tx("fall");
    fall = false;
  }

  LoraSerial.end();

  delay(3000);
}



// -------------------- FUNCTIONS --------------------


// LoRa initialization
void initialize_radio()
{
  // Reset RN2xx3
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);

  delay(100); // Wait for the RN2xx3's startup message
  LoraSerial.flush();

  // Check communication with radio
  String hweui = myLora.hweui();
  while (hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  //Serial.println("When using OTAA, register this DevEUI: ");
  //Serial.println(hweui);
  //Serial.println("RN2xx3 firmware version:");
  //Serial.println(myLora.sysver());

  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  //OTAA: initOTAA(String AppEUI, String AppKey);
  join_result = myLora.initOTAA("BE7A000000001296", "12DCEE56CD95013A24B6C8B31CE7032E");

  while (!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");

}


// Get GPS coordinates
void getGPSInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);

    dtostrf(gps.location.lat(), 10, 6, latitude); // store latitude in char array
    dtostrf(gps.location.lng(), 10, 6, longitude); // store longitude in char array
  }
  else
  {
    Serial.print(F("INVALID"));
    strcpy(latitude, "0.0");
    strcpy(longitude, "0.0");
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
