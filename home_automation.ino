
// ESP32 Google Assistant + Alexa + Manual HomeAutomation with Sinric Pro


#include <Arduino.h>                          //ArduinoJson Library: https://github.com/bblanchon/ArduinoJson
#include <WiFi.h>
#include "SinricPro.h"                        //SinricPro Library: https://sinricpro.github.io/esp8266-esp32-sdk/
#include "SinricProSwitch.h"
#include <map>
#include "DHT.h"

// include the library code:
#include <LiquidCrystal.h>
 


// initialize the library with the numbers of the interface pins

// Uncomment the following line to enable serial debug output
//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
       #define DEBUG_ESP_PORT Serial
       #define NODEBUG_WEBSOCKETS             //arduinoWebSockets Library: https://github.com/Links2004/arduinoWebSockets
       #define NDEBUG
#endif 
#define WIFI_SSID         "tarapdpo30"      //Enter WiFi Name
#define WIFI_PASS         "CLB2838836"      //Enter WiFi Password
#define APP_KEY           "7ab0cd90-4d3d-474d-a83a-20c0ae476c82"                  //Enter APP-KEY 
#define APP_SECRET        "e904ec90-1a05-4d67-8b16-f5be38b5fd02-be821aa2-f79f-4ceb-ac34-06b585e70d41"     //Enter APP-SECRET

//Enter the device IDs here
#define device_ID_1   "63878a02b8a7fefbd64f4a72"  //SWITCH 1 ID
#define device_ID_2   "63878a1b333d12dd2a03ae57"  //SWITCH 2 ID
#define device_ID_3   "63878a38b8a7fefbd64f4b34"  //SWITCH 3 ID

// define the GPIO connected with Relays and switches
#define RelayPin1 15  //D15
#define RelayPin2 2  //D2
#define RelayPin3 4  //D4
#define RelayPinBuzzer 13 //D13

#define SwitchPin1 23  //D23
#define SwitchPin2 22  //D22
#define SwitchPin3 21  //D21


#define wifiLed   2   //D2


//display







#define BAUD_RATE   9600

#define DEBOUNCE_TIME 250

typedef struct {      // struct for the std::map below
  int relayPIN;
  int flipSwitchPIN;
} deviceConfig_t;


std::map<String, deviceConfig_t> devices = {
    //{deviceId, {relayPIN,  flipSwitchPIN}}
    {device_ID_1, {  RelayPin1, SwitchPin1 }},
    {device_ID_2, {  RelayPin2, SwitchPin2 }},
    {device_ID_3, {  RelayPin3, SwitchPin3 }},
         
};

typedef struct {      // struct for the std::map below
  String deviceId;
  bool lastFlipSwitchState;
  unsigned long lastFlipSwitchChange;
} flipSwitchConfig_t;

std::map<int, flipSwitchConfig_t> flipSwitches;    // this map is used to map flipSwitch PINs to deviceId and handling debounce and last flipSwitch state checks
                                                  // it will be setup in "setupFlipSwitches" function, using informations from devices map

void setupRelays() { 
  for (auto &device : devices) {           // for each device (relay, flipSwitch combination)
    int relayPIN = device.second.relayPIN; // get the relay pin
    pinMode(relayPIN, OUTPUT);             // set relay pin to OUTPUT
    digitalWrite(relayPIN, HIGH);
  }
}

void setupFlipSwitches() {
  for (auto &device : devices)  {                     // for each device (relay / flipSwitch combination)
    flipSwitchConfig_t flipSwitchConfig;              // create a new flipSwitch configuration

    flipSwitchConfig.deviceId = device.first;         // set the deviceId
    flipSwitchConfig.lastFlipSwitchChange = 0;        // set debounce time
    flipSwitchConfig.lastFlipSwitchState = false;     // set lastFlipSwitchState to false (LOW)--

    int flipSwitchPIN = device.second.flipSwitchPIN;  // get the flipSwitchPIN

    flipSwitches[flipSwitchPIN] = flipSwitchConfig;   // save the flipSwitch config to flipSwitches map
    pinMode(flipSwitchPIN, INPUT_PULLUP);                   // set the flipSwitch pin to INPUT
  }
}

bool onPowerState(String deviceId, bool &state)
{
  Serial.printf("%s: %s\r\n", deviceId.c_str(), state ? "on" : "off");
  int relayPIN = devices[deviceId].relayPIN; // get the relay pin for corresponding device
  digitalWrite(relayPIN, !state);             // set the new relay state
  return true;
}

void handleFlipSwitches() {
  unsigned long actualMillis = millis();                                          // get actual millis
  for (auto &flipSwitch : flipSwitches) {                                         // for each flipSwitch in flipSwitches map
    unsigned long lastFlipSwitchChange = flipSwitch.second.lastFlipSwitchChange;  // get the timestamp when flipSwitch was pressed last time (used to debounce / limit events)

    if (actualMillis - lastFlipSwitchChange > DEBOUNCE_TIME) {                    // if time is > debounce time...

      int flipSwitchPIN = flipSwitch.first;                                       // get the flipSwitch pin from configuration
      bool lastFlipSwitchState = flipSwitch.second.lastFlipSwitchState;           // get the lastFlipSwitchState
      bool flipSwitchState = digitalRead(flipSwitchPIN);                          // read the current flipSwitch state
      if (flipSwitchState != lastFlipSwitchState) {                               // if the flipSwitchState has changed...
#ifdef TACTILE_BUTTON
        if (flipSwitchState) {                                                    // if the tactile button is pressed 
#endif      
          flipSwitch.second.lastFlipSwitchChange = actualMillis;                  // update lastFlipSwitchChange time
          String deviceId = flipSwitch.second.deviceId;                           // get the deviceId from config
          int relayPIN = devices[deviceId].relayPIN;                              // get the relayPIN from config
          bool newRelayState = !digitalRead(relayPIN);                            // set the new relay State
          digitalWrite(relayPIN, newRelayState);                                  // set the trelay to the new state

          SinricProSwitch &mySwitch = SinricPro[deviceId];                        // get Switch device from SinricPro
          mySwitch.sendPowerStateEvent(!newRelayState);                            // send the event
#ifdef TACTILE_BUTTON
        }
#endif      
        flipSwitch.second.lastFlipSwitchState = flipSwitchState;                  // update lastFlipSwitchState
      }
    }
  }
}

void setupWiFi()
{
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.printf(".");
    delay(250);
  }
  digitalWrite(wifiLed, HIGH);
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %s\r\n", WiFi.localIP().toString().c_str());
}

void setupSinricPro()
{
  for (auto &device : devices)
  {
    const char *deviceId = device.first.c_str();
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.onPowerState(onPowerState);
  }

  SinricPro.begin(APP_KEY, APP_SECRET);
  SinricPro.restoreDeviceStates(true);
}

//dht sensor
#define DHTTYPE DHT11   // DHT 11

uint8_t DHTPin = 5; 

DHT dht(DHTPin, DHTTYPE);                

float Temperature;
float Humidity;

 int trigPin = 18;
 int echoPin = 19;
 int duration;
 int distance;
void setup()
{
  Serial.begin(BAUD_RATE);
  delay(100);

  pinMode(wifiLed, OUTPUT);
  pinMode(DHTPin, INPUT);

  digitalWrite(wifiLed, LOW);

  setupRelays();
  setupFlipSwitches();
  setupWiFi();
  setupSinricPro();

  
  dht.begin();              

  Serial.println("Reading temperature and humidity ");


  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
   pinMode(RelayPinBuzzer, OUTPUT);
 digitalWrite(RelayPinBuzzer,HIGH);

   
}
int period = 2000;
unsigned long time_now = 0;
int period_buzzer = 2000;
unsigned long time_now_buzzer = 0;
 
//ultrasonic sensor


void loop()
{
  SinricPro.handle();
  handleFlipSwitches();
   if(millis() >= time_now + period){
        time_now += period;
 
  Temperature = dht.readTemperature(); // Gets the values of the temperature
    Humidity = dht.readHumidity(); // Gets the values of the humidity 
  Serial.println("Temperature:\n");
  Serial.println(Temperature);
  Serial.println("Humidity:\n");
  Serial.println(Humidity);
    }
     digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance= duration*0.034/2;
 
  if(distance<=10){
    digitalWrite(RelayPinBuzzer,LOW);

  }
  else if(millis() >= time_now_buzzer + period_buzzer){
        time_now_buzzer += period_buzzer;
 digitalWrite(RelayPinBuzzer,HIGH);
    }
  delay(100);

}

