<div align="center"><p>
    <a href="https://github.com/bulb-light/PID_lib_dff/pulse">
      <img alt="Last commit" src="https://img.shields.io/github/last-commit/bulb-light/PID_lib_dff?style=for-the-badge&logo=starship&color=8bd5ca&logoColor=D9E0EE&labelColor=302D41"/>
    </a>
    <a href="https://github.com/bulb-light/PID_lib_dff/blob/master/LICENSE">
      <img alt="License" src="https://img.shields.io/github/license/bulb-light/PID_lib_dff?style=for-the-badge&logo=starship&color=ee999f&logoColor=D9E0EE&labelColor=302D41" />
    </a>
    <a href="https://github.com/bulb-light/PID_lib_dff/stargazers">
      <img alt="Stars" src="https://img.shields.io/github/stars/bulb-light/PID_lib_dff?style=for-the-badge&logo=starship&color=c69ff5&logoColor=D9E0EE&labelColor=302D41" />
    </a>
    <a href="https://github.com/bulb-light/PID_lib_dff/issues">
      <img alt="Issues" src="https://img.shields.io/github/issues/bulb-light/PID_lib_dff?style=for-the-badge&logo=bilibili&color=F5E0DC&logoColor=D9E0EE&labelColor=302D41" />
    </a>
    <a href="https://github.com/bulb-light/PID_lib_dff">
      <img alt="Repo Size" src="https://img.shields.io/github/repo-size/bulb-light/PID_lib_dff?color=%23DDB6F2&label=SIZE&logo=codesandbox&style=for-the-badge&logoColor=D9E0EE&labelColor=302D41" />
    </a>
    <a href="https://www.youtube.com/@david-dff-bulblight">
      <img alt="follow on Youtube" src="https://img.shields.io/twitter/follow/david-dff-bulblight?style=for-the-badge&logo=youtube&color=8aadf3&logoColor=D9E0EE&labelColor=302D41" />
    </a>
</div>

# PID Library (Generic implementation of the algorithm in C++)

## Overview
This library provides a discrete PID (Proportional-Integral-Derivative) controller [implementation](DiscreteTimePID.md) in C++. Compatible with Arduino and ESP32 boards. The codebase is written in standard C++ to remain generic across different microcontrollers.

Experimental library. Minimal tested, so usage remarks and comments are welcome.

### Consult the documentation for the mathematical background of the discrete PID implementation:
[![Discrete-time PID controller](https://img.shields.io/badge/Discrete%20Time%20PID-Go-Color?style=for-the-badge)](DiscreteTimePID.md)

## Features
- Discrete PID controller implementation using:
  - Trapezoidal approximation for the integral term.
  - Backward-difference approximation for the derivative term.
- Dynamic reconfiguration of PID parameters.
- Adjustable sampling time.
- Output clamping with anti-windup.
- Reset functionality to clear internal states.

## Installation
1. Copy the `PID.h` and `PID.cpp` files into your Arduino project's `lib/PID/` directory if you are using platformio in vscode. If you are using Arduino IDE 2, copy the files to `C:\Users\user\AppData\Local\Arduino15\libraries\PID_lib_dff\`or to your Arduino libraries folder.
2. Alternatively, clone the repository directly into your libraries folder using the following command:

   ```bash
   git clone https://github.com/bulb-light/PID_lib_dff.git <path_to_libraries_folder>
   ```
3. If you intend to use the library with other microcontrollers, ensure you configure it correctly for your specific environment.
4. Include the library in your project using `#include <PID.h>`.

## API Reference

### Constructor
```cpp
PID(float Kp, float Ti, float Td, float sampleTimeSec, float minOutput, float maxOutput);
```
- **Kp**: Proportional gain.
- **Ti**: Integral time constant (in seconds). Defaults to `0.0001` to avoid division by zero if invalid.
- **Td**: Derivative time constant (in seconds).
- **sampleTimeSec**: Sampling time (in seconds). Defaults to `0.01` if invalid (if `value <= 0`).
- **minOutput**: Minimum output limit. Defaults to `0.0` if invalid.
- **maxOutput**: Maximum output limit. Defaults to `255.0` if invalid (`minOutput >= maxOutput`).

### Methods

#### `void setSampleTime(float sampleTimeSec)`
Updates the sampling time and recalculates PID coefficients.

#### `void setPIDParams(float Kp, float Ti, float Td)`
Updates the PID parameters dynamically and recalculates coefficients.

#### `void setOutputLimits(float minOutput, float maxOutput)`
Sets new output limits for the PID controller.

#### `float computePIDOut(float error)`
Computes the PID output based on the current error.

#### `void resetStates()`
Resets the internal states of the PID controller.

## Example Usage
Below is an example of using the PID library to control the temperature of a system using an Arduino Nano and an LM35 temperature sensor (check the full project on [TemperatureControlLM35](https://github.com/bulb-light/ArduinoProjects_dff/tree/main/TemperatureControlLM35)).

```cpp
#include <Arduino.h>
#include <PID.h>

// Sensor and actuator pins
#define LM35_PIN A0
#define TIP31C_PIN 3

// PID parameters
float Kp = 2.0;
float Ti = 40.0;
float Td = 5.0;
float sampleTimeSec = 0.5;
float minOutput = 0.0;
float maxOutput = 255.0;
PID tempPID(Kp, Ti, Td, sampleTimeSec, minOutput, maxOutput);

// Control variables
float temp_ref = 50.0; // Desired temperature
long prev_millis_temp = 0;
long control_interval = (int)(1000 * sampleTimeSec);

void setup() {
  Serial.begin(115200);
  pinMode(LM35_PIN, INPUT);
  pinMode(TIP31C_PIN, OUTPUT);
  tempPID.resetStates();
}

void loop() {
  int sensor_value = analogRead(LM35_PIN);
  float voltage = sensor_value * (5.0 / 1023.0);
  float temperature_c = voltage * 100.0;

  long current_millis = millis();
  if (current_millis - prev_millis_temp >= control_interval) {
    prev_millis_temp = current_millis;

    float error = temp_ref - temperature_c;
    float pid_output = tempPID.computePIDOut(error);

    analogWrite(TIP31C_PIN, (int)pid_output);

    Serial.print("Setpoint: ");
    Serial.print(temp_ref);
    Serial.print(" °C, Temperature: ");
    Serial.print(temperature_c);
    Serial.print(" °C, PID Output: ");
    Serial.println(pid_output);
  }
}
```

## Notes
- Ensure that the `computePIDOut()` method is called at a fixed interval defined by the sampling time.
- The library assumes that the system being controlled is sampled at regular intervals.

## License
This library is open-source and available under the MIT License.


---

## 📫 How to Reach Me

| Platform | Handle / Link |
|---|---|
| Email | davidcs.ee.10@gmail.com |
| LinkedIn | [david](https://www.linkedin.com/in/davidcsee/) |
| Tiktok | [david_dff_bulblight](https://www.tiktok.com/@david_dff_bulblight)|
| YouTube| [david-dff](https://www.youtube.com/@david-dff-bulblight)|

---

## 🔗 Connect & Collaborate

I’m open to collaboration on open source, side projects, or mentoring.  
Feel free to reach out!

If you appreciate my work, you can support its development and maintenance. Improve the quality of the libraries by providing issues and Pull Requests, or make a donation.

Thank you.
