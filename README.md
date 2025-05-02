# IoT_MQTT
With this project, you can control a servo motor using MQTT protocol and monitor temperature and humidity using a DHT11 sensor. 
Simply follow the board setup and clone this repository or copy ```main.cpp```.

## Board Setup
![img.png](images/img.png)

## Components used:
| Component             | Description                                |
| --------------------- |--------------------------------------------|
| Arduino MKR WiFi 1010 | Microcontroller board with WiFi capability |
| DHT11 Sensor          | Measures temperature and humidity          |
| Servo Motor (SG90)    | Small servo for mechanical movement        |
| Breadboard            | Used to connect and prototype the circuit  |
| Jumper Wires          | For making the connections                 |
| USB Cable             | To power and upload code to the Arduino    |

## Circuit Diagram
**DHT11 Sensor:**
- **VCC** (red wire) → 5V
- **GND** (black wire) → GND
- **DATA** (orange wire) → Any Digital Pin _(Demo uses **D7**)_

**Servo Motor:**
- **VCC** (red wire) → 5V
- **GND** (black wire) → GND
- **Signal** (orange wire) → Any Analog Pin with PWM _(Demo uses **A3**)_

## Example of arduino.secrets.h
Use this example as a template for setting up your own `arduino_secrets.h` file.
```cpp
#define SECRET_SSID "Network name"
#define SECRET_PASS "Network password"

#define BROKER "BROKER"
#define CLIENT_ID "CLIENT_ID"
#define HIVEMQ_USERNAME "MQTTpub"
#define HIVEMQ_PASSWORD "P@ssw0rd"
```


## Libraries used
- `DHT.h` - For DHT11 temperature and humidity sensor
- `Servo.h` - For controlling the servo motor
- `ArduinoMqttClient.h` - For MQTT communication
- `WiFiNINA.h` - For WiFi connectivity on MKR WiFi 1010 board
- `Arduino.h` - Core Arduino functionality
- `arduino_secrets.h` - Local header for credentials (not a library, but a project file)