# ESP32 Voice-Controlled Home Automation System

This project implements a voice-controlled home automation system using an ESP32 microcontroller, integrated with Google Assistant, Alexa, and manual controls via Sinric Pro. It also includes environmental monitoring with a DHT11 sensor and proximity detection with an ultrasonic sensor and buzzer.

## Features
- **Voice Control**: Control home appliances using Google Assistant or Alexa via Sinric Pro.
- **Manual Control**: Toggle appliances using physical switches connected to the ESP32.
- **Environmental Monitoring**: Measure temperature and humidity with a DHT11 sensor.
- **Proximity Detection**: Trigger a buzzer when an object is detected within 10 cm using an ultrasonic sensor.
- **WiFi Connectivity**: Connects to a WiFi network for cloud-based control.
- **Sinric Pro Integration**: Manages device states and remote control through the Sinric Pro platform.

## Hardware Requirements
- ESP32 microcontroller
- DHT11 temperature and humidity sensor (connected to GPIO 5)
- Ultrasonic sensor (HC-SR04, trigger: GPIO 18, echo: GPIO 19)
- Buzzer (connected to GPIO 13)
- Relays for appliance control (GPIO 15, 2, 4)
- Push-button switches for manual control (GPIO 23, 22, 21)
- WiFi network for connectivity

## Software Requirements
- Arduino IDE with ESP32 board support
- Libraries:
  - [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
  - [SinricPro](https://sinricpro.github.io/esp8266-esp32-sdk/)
  - [arduinoWebSockets](https://github.com/Links2004/arduinoWebSockets)
  - [DHT sensor library](https://github.com/adafruit/DHT-sensor-library)

## Setup Instructions
1. **Install Libraries**: Install the required libraries in the Arduino IDE.
2. **Configure WiFi and Sinric Pro**:
   - Update `WIFI_SSID` and `WIFI_PASS` with your WiFi credentials.
   - Set `APP_KEY` and `APP_SECRET` from your Sinric Pro account.
   - Update `device_ID_1`, `device_ID_2`, and `device_ID_3` with your Sinric Pro device IDs.
3. **Connect Hardware**:
   - Connect relays to GPIO 15, 2, and 4 for appliance control.
   - Connect switches to GPIO 23, 22, and 21 for manual toggling.
   - Connect the DHT11 sensor to GPIO 5.
   - Connect the ultrasonic sensor to GPIO 18 (trigger) and GPIO 19 (echo).
   - Connect the buzzer to GPIO 13.
4. **Upload Code**: Upload the provided C++ code to the ESP32 using the Arduino IDE.
5. **Sinric Pro Setup**:
   - Configure devices in the Sinric Pro app or web interface.
   - Link Sinric Pro with Google Assistant or Alexa for voice control.

## Code Overview
- **WiFi Setup**: Connects to the specified WiFi network and indicates status via an LED on GPIO 2.
- **Relay Control**: Manages three relays for appliances, toggled via Sinric Pro or manual switches.
- **Switch Handling**: Debounces manual switch inputs with a 250ms delay.
- **DHT11 Sensor**: Reads temperature and humidity every 2 seconds and prints to Serial.
- **Ultrasonic Sensor**: Measures distance every loop cycle; triggers the buzzer if an object is within 10 cm.
- **Buzzer Control**: Activates the buzzer for proximity alerts, with a 2-second reset period.

## Pin Configuration
- Relays: GPIO 15 (Relay 1), GPIO 2 (Relay 2), GPIO 4 (Relay 3)
- Switches: GPIO 23 (Switch 1), GPIO 22 (Switch 2), GPIO 21 (Switch 3)
- DHT11: GPIO 5
- Ultrasonic Sensor: GPIO 18 (Trigger), GPIO 19 (Echo)
- Buzzer: GPIO 13
- WiFi LED: GPIO 2

## Usage
- **Voice Control**: Use Google Assistant or Alexa to turn devices on/off via Sinric Pro.
- **Manual Control**: Press the physical switches to toggle appliances.
- **Monitoring**: Check the Serial Monitor (9600 baud) for temperature and humidity readings.
- **Proximity Alert**: The buzzer activates when an object is detected within 10 cm.

## Notes
- Ensure stable WiFi for reliable voice control.
- Verify Sinric Pro device IDs and credentials for proper cloud integration.
- The ultrasonic sensor and buzzer provide basic proximity alerts; adjust the distance threshold (`distance <= 10`) as needed.
