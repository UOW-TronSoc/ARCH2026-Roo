# Roo – ESP32 Low-Level Controller

ESP32 firmware for low level control of Roo.  
It controls gimbal + suspension servos, two IBT-4 motor drivers, reads two MT6701 PWM encoders, and reads an INA226 over I2C.  
A Raspberry Pi communicates with the ESP32 over **USB serial** using a simple framed message protocol.

---

## Serial Protocol

All messages use:

$ID~message`

- `$` = start of frame  
- `<ID>` = numeric message/device ID  
- `~` = separator  
- `<message>` = payload string  
- `` ` `` = end of frame
Example: $3~Hi`

| Component/System | Device ID | Message Format | Notes |
|---------|----------|----------|------|
| ** RaspberryPi > ESP32 |
| Gimbal Servo 1 | 1 | Angle in degrees | Position servos, 0–180° |
| Gimbal Servo 2 | 2 | Angle in degrees | Position servos, 0–180° |
| Gimbal Servo 3 | 3 | Angle in degrees  | Position servos, 0–180° |
| Suspension Servo Front | 4 | speed -10-+10 | Servo speed, -10–+10 |
| Suspension Servo Back | 5 | speed -10-+10 | Servo speed, -10–+10 |
| Motor Driver Left | 6 | speed -10-+10 | Servo speed, -10–+10 |
| Motor Driver Right | 7 | speed -10-+10 | Servo speed, -10–+10 |
| ESP32 > RaspberryPi |
| INA226 | 10 | V<voltage>-I<current> | Absolute angles of suspensions |
| Suspension Encoders | 11 | F<Angle>B<Angle> | Absolute angles of suspensions |

---

## Hardware

### Pin Assignments
| Device | ESP32 Pin | Component Pin | Description |
|---------|----|------|------|
| Gimbal Servo 1 | G13 | SIG | PWM Control |
| Gimbal Servo 2 | G12 | SIG | PWM Control |
| Gimbal Servo 3 | G14 | SIG | PWM Control |
| Suspension Servo Front | G33 | SIG | PWM Control |
| Suspension Servo Back| G32 | SIG | PWM Control |
| Motor Driver Left | G17 | IN1 | PWM Control |
| Motor Driver Left | G16 | IN2 | PWM Control |
| Motor Driver Right | G19 | IN1 | PWM Control |
| Motor Driver Right | G18 | IN2 | PWM Control |
| Suspension Encoder Front | G39 | OUT | PWM Input |
| Suspension Encoder Back | G36 | OUT | PWM Input |
| INA226 | G21 | SDA | I2C Control |
| INA226 | G22 | SCL | I2C Control |


### Actuators
- **3× Gimbal servos** (position, 0–180°)
- **2× Suspension servos** (continuous rotation)
- **2× IBT-4 motor drivers** (each uses 2 PWM pins: forward + reverse)

### Sensors
- **2× MT6701 absolute encoders** in **PWM output mode** (measured via interrupt timing)
- **1× INA226** current/voltage sensor over **I2C**

---

## Telemetry to Raspberry Pi

### INA226 Telemetry (ID 10)
Sent at **2 Hz** (soft-coded interval).

Payload format:
V12.00-I1.10
Where:
- `V` = bus voltage in volts
- `I` = current in amps

Example frame:
$10~V12.34-I0.56`


### Encoder Telemetry (ID 11)
Sent at a configured interval (soft-coded).

Payload format:
F12B23
Where:
- `F` = front encoder absolute angle (degrees)
- `B` = back encoder absolute angle (degrees)

Example frame:
$11~F180B92`


---

## Suspension Software Limits

Each suspension servo has **min/max angle limits** (degrees) based on its corresponding encoder.

- If the encoder reaches **maxDeg**, the servo is prevented from driving further in the “positive” direction.
- If the encoder reaches **minDeg**, the servo is prevented from driving further in the “negative” direction.
- It can still move away from the limit in the opposite direction.

---
