#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define RESET 15

static const int RX_GPS = 6, TX_GPS = 5;
static const int RX_Lora = 4, TX_Lora = 3;
static const uint32_t SerialMonitorBaud = 115200;
static const uint32_t GPSBaud = 9600;
static const uint32_t LoraBaud = 57600;

char latitude[15];
char longitude[15];
char message[35];
bool fall = false;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// The TinyGPS++ object
TinyGPSPlus gps;

SoftwareSerial GPSSerial(RX_GPS, TX_GPS); // Serial connection to the GPS device
SoftwareSerial LoraSerial(RX_Lora, TX_Lora); // Serial connection to the Lora device

//create an instance of the rn2xx3 library,
//giving the software UART as stream to use,
//and using LoRa WAN
rn2xx3 myLora(LoraSerial);

// the setup routine runs once when you press reset:
void setup() {

  if (!accel.begin())
  {
    Serial.println("No valid sensor found");
    while (1);
  }

  // LED pin is GPIO2 which is the ESP8266's built in LED
  pinMode(LED_BUILTIN, OUTPUT);
  led_on();

  // Open serial communications and wait for port to open:
  Serial.begin(SerialMonitorBaud);
  LoraSerial.begin(LoraBaud);

  delay(1000); //wait for the arduino ide's serial console to open

  Serial.println("Startup");

  initialize_radio();

  //transmit a startup message
  //myLora.tx("TTN Mapper on Uno node");
  LoraSerial.end();

  led_off();
  delay(3000);
}


// Lora
void initialize_radio()
{
  //reset RN2xx3
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  LoraSerial.flush();

  //check communication with radio
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


// the loop routine runs over and over again forever:
void loop() {

  sensors_event_t event;
  accel.getEvent(&event);
  float g = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
  if (g > 20) {
    Serial.println("Fall");
    fall = true;
  }

  GPSSerial.begin(GPSBaud);
  delay(1000);

  // This sketch displays information every time a new sentence is correctly encoded.
  while (GPSSerial.available() > 0)
    if (gps.encode(GPSSerial.read()))
      displayGPSInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }

  GPSSerial.end();
  delay(1000);

  led_on();

  LoraSerial.begin(LoraBaud);
  delay(1000);
  myLora.sendRawCommand(F("mac set adr on"));
  Serial.println("OK");
  delay(500);

  myLora.tx(message); //one byte, blocking function
  //send fall if falling
  if (fall) {
    myLora.tx("fall");
    fall = false;
  }

  LoraSerial.end();

  led_off();

  delay(3000);
}


// GPS
void displayGPSInfo()
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

  // prepare message:
  strcpy(message, latitude);
  strcat(message, ",");
  strcat(message, longitude);
  //Serial.print("message: ");
  //Serial.println(message);

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


// builtin LED
void led_on()
{
  digitalWrite(LED_BUILTIN, 1);
}

void led_off()
{
  digitalWrite(LED_BUILTIN, 0);
}
