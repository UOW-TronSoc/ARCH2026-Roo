/*
  Roo ESP32 Low-Level Controller (ESP32 DevKit 38-pin, Arduino IDE)

  Serial protocol: $ID~message`

  Motors: PROPORTIONAL SPEED using IN1/IN2 PWM logic:
    val > 0  => forward (IN1=PWM, IN2=0)
    val < 0  => reverse (IN1=0,   IN2=PWM)
    val = 0  => stop    (IN1=0,   IN2=0)

  Incoming command frames:
    ID 1: Gimbal Servo 1 CR servo speed (-10..+10)
    ID 2: Gimbal Servo 2 CR servo speed (-10..+10)
    ID 3: Gimbal Servo 3 CR servo speed (-10..+10)
    ID 4: Suspension Front CR servo speed (-10..+10)
    ID 5: Suspension Back  CR servo speed (-10..+10)
    ID 6: Left  motor command (-10..+10)
    ID 7: Right motor command (-10..+10)

    ID 8: NIR Spectral Triad trigger:
      "$8~1`" => capture 18 calibrated channels and reply on ID 13

  Telemetry:
    ID 12: Tilt from GY-85 (ADXL345 accel) => "$12~P<ANGLE>R<ANGLE>`" @ 4 Hz

  NIR reply:
    ID 13: "$13~A<V>,B<V>,C<V>...`"
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <math.h>
#include <INA226_WE.h>

#include "SparkFun_AS7265X.h"   // Spectral Triad (AS7265x)

/* ===================== CONFIG ===================== */

// Serial
static const uint32_t SERIAL_BAUD = 115200;

// I2C pins
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// Gimbal servos (now continuous rotation)
static const int PIN_GIMBAL1 = 13;
static const int PIN_GIMBAL2 = 12;
static const int PIN_GIMBAL3 = 14;

// Suspension servos (continuous rotation)
static const int PIN_SUSP_F  = 33;
static const int PIN_SUSP_B  = 32;

// Servo pulse bounds
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2400;

// Continuous rotation mapping
static const int CR_STOP_US = 1500;
static const int CR_US_PER_SPEED = 35;  // tune for your CR servos

// Motors (IN1 / IN2)
static const int PIN_MOT_L_IN1 = 17;
static const int PIN_MOT_L_IN2 = 16;
static const int PIN_MOT_R_IN1 = 19;
static const int PIN_MOT_R_IN2 = 18;

// Telemetry rates
static const uint32_t TILT_PERIOD_MS = 250;  // 4 Hz
static const uint32_t POWER_PERIOD_MS = 20;  // 50 Hz

static const uint8_t ADXL345_ADDR = 0x53;

static const int PIN_ENC1 = 36; 
static const int PIN_ENC2 = 39;

static float enc1_deg = 0.0;
static float enc2_deg = 0.0;

INA226_WE ina226 = INA226_WE(0x45); 

static float current_ma = 0.0;
static float bus_v = 0.0;
static float accumulated_energy_mwh = 0.0;
static unsigned long last_energy_update = 0;
static bool is_recording_energy = false;

/* ===================== UTILS ===================== */

static inline int clampi(int x, int lo, int hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// Replaced isSignedInt with isNumeric to allow float parsing
static bool isNumeric(const String& s) {
  if (s.length() == 0) return false;
  int i = 0;
  if (s[0] == '+' || s[0] == '-') {
    if (s.length() == 1) return false;
    i = 1;
  }
  bool hasDot = false;
  for (; i < s.length(); i++) {
    char c = s[i];
    if (c == '.') {
      if (hasDot) return false;
      hasDot = true;
    } else if (c < '0' || c > '9') {
      return false;
    }
  }
  return true;
}

static void sendFrame(uint8_t id, const String& payload) {
  Serial.print('$');
  Serial.print(id);
  Serial.print('~');
  Serial.print(payload);
  Serial.print('`');
}

/* ===================== MOTORS (PROPORTIONAL SPEED) ===================== */

static void motorsInit() {
  // ESP32 Arduino Core 3.x.x API
  ledcAttach(PIN_MOT_L_IN1, 5000, 8);
  ledcAttach(PIN_MOT_L_IN2, 5000, 8);
  ledcAttach(PIN_MOT_R_IN1, 5000, 8);
  ledcAttach(PIN_MOT_R_IN2, 5000, 8);

  // Stop on boot
  ledcWrite(PIN_MOT_L_IN1, 0); 
  ledcWrite(PIN_MOT_L_IN2, 0);
  ledcWrite(PIN_MOT_R_IN1, 0); 
  ledcWrite(PIN_MOT_R_IN2, 0);

  Serial.println("[MOTOR] PWM proportional speed control ready");
}

static void motorLeftWrite(float val) {
  // Map -10..+10 to 0..255
  int speed = clampi((int)(fabs(val) * 25.5f), 0, 255);
  if (val > 0.01) {
    ledcWrite(PIN_MOT_L_IN1, speed);
    ledcWrite(PIN_MOT_L_IN2, 0);
  } else if (val < -0.01) {
    ledcWrite(PIN_MOT_L_IN1, 0);
    ledcWrite(PIN_MOT_L_IN2, speed);
  } else {
    ledcWrite(PIN_MOT_L_IN1, 0);
    ledcWrite(PIN_MOT_L_IN2, 0);
  }
}

static void motorRightWrite(float val) {
  int speed = clampi((int)(fabs(val) * 25.5f), 0, 255);
  if (val > 0.01) {
    ledcWrite(PIN_MOT_R_IN1, speed);
    ledcWrite(PIN_MOT_R_IN2, 0);
  } else if (val < -0.01) {
    ledcWrite(PIN_MOT_R_IN1, 0);
    ledcWrite(PIN_MOT_R_IN2, speed);
  } else {
    ledcWrite(PIN_MOT_R_IN1, 0);
    ledcWrite(PIN_MOT_R_IN2, 0);
  }
}

/* ===================== SERVOS ===================== */

static Servo g1, g2, g3;
static Servo suspF, suspB;

static void crServoWriteSpeed(Servo &s, float speed10) {
  // Safely clamp float speed and convert to microseconds
  speed10 = (speed10 < -10.0f) ? -10.0f : (speed10 > 10.0f) ? 10.0f : speed10;
  int us = CR_STOP_US + (int)(speed10 * CR_US_PER_SPEED);
  us = clampi(us, SERVO_MIN_US, SERVO_MAX_US);
  s.writeMicroseconds(us);
}

/* ===================== NIR (AS7265X Spectral Triad) ===================== */

static AS7265X nir;
static bool nirOk = false;

static bool nirInit() {
  if (nir.begin() == false) return false;
  nir.disableIndicator();
  return true;
}

static void sendNIRReadingFrame() {
  if (!nirOk) return;
  nir.takeMeasurementsWithBulb();
  String data;
  data.reserve(200);

  data += "A"; data += String(nir.getCalibratedA()); data += ",";
  data += "B"; data += String(nir.getCalibratedB()); data += ",";
  data += "C"; data += String(nir.getCalibratedC()); data += ",";
  data += "D"; data += String(nir.getCalibratedD()); data += ",";
  data += "E"; data += String(nir.getCalibratedE()); data += ",";
  data += "F"; data += String(nir.getCalibratedF()); data += ",";
  data += "G"; data += String(nir.getCalibratedG()); data += ",";
  data += "H"; data += String(nir.getCalibratedH()); data += ",";
  data += "R"; data += String(nir.getCalibratedR()); data += ",";
  data += "I"; data += String(nir.getCalibratedI()); data += ",";
  data += "S"; data += String(nir.getCalibratedS()); data += ",";
  data += "J"; data += String(nir.getCalibratedJ()); data += ",";
  data += "T"; data += String(nir.getCalibratedT()); data += ",";
  data += "U"; data += String(nir.getCalibratedU()); data += ",";
  data += "V"; data += String(nir.getCalibratedV()); data += ",";
  data += "W"; data += String(nir.getCalibratedW()); data += ",";
  data += "K"; data += String(nir.getCalibratedK()); data += ",";
  data += "L"; data += String(nir.getCalibratedL());

  sendFrame(13, data);
}

/* ===================== SERIAL PARSER ===================== */

static String rxBuf;

static void handleFrame(uint8_t id, const String &payload) {
  String p = payload;
  p.trim();

  if (id == 8) {
    if (p == "1") sendNIRReadingFrame();
    return;
  }
  if (id == 20) {
    if (p == "1") {
      accumulated_energy_mwh = 0.0;
      is_recording_energy = true;
    } else {
      is_recording_energy = false;
    }
    last_energy_update = millis();
    return;
  }

  if (id >= 1 && id <= 7) {
    if (!isNumeric(p)) return;
  }

  // Use toFloat to read the exact decimal representation
  float val = p.toFloat();

  switch (id) {
    case 1: crServoWriteSpeed(g1, val); break;
    case 2: crServoWriteSpeed(g2, val); break;
    case 3: crServoWriteSpeed(g3, val); break;

    case 4: crServoWriteSpeed(suspF, val); break;
    case 5: crServoWriteSpeed(suspB, val); break;

    case 6: motorLeftWrite(val);  break;
    case 7: motorRightWrite(val); break;

    default: break;
  }
}

static void parseSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '$') { rxBuf = "$"; continue; }
    if (rxBuf.length() == 0) continue;

    rxBuf += c;

    if (c == '`') {
      int tilde = rxBuf.indexOf('~');
      if (tilde > 1) {
        uint8_t id = (uint8_t)rxBuf.substring(1, tilde).toInt();
        String payload = rxBuf.substring(tilde + 1, rxBuf.length() - 1);
        handleFrame(id, payload);
      }
      rxBuf = "";
    }

    if (rxBuf.length() > 256) rxBuf = "";
  }
}

/* ===================== GY-85 (ADXL345) PITCH/ROLL ===================== */

static int pitchDeg = 0;
static int rollDeg  = 0;

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cRead(uint8_t addr, uint8_t startReg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool adxl345Init() {
  uint8_t id = 0;
  if (!i2cRead(ADXL345_ADDR, 0x00, &id, 1)) return false;
  if (id != 0xE5) return false;

  if (!i2cWrite8(ADXL345_ADDR, 0x2D, 0x08)) return false;
  if (!i2cWrite8(ADXL345_ADDR, 0x31, 0x08)) return false;
  i2cWrite8(ADXL345_ADDR, 0x2C, 0x0A);
  return true;
}

static bool adxl345ReadRaw(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t b[6];
  if (!i2cRead(ADXL345_ADDR, 0x32, b, 6)) return false;
  x = (int16_t)((b[1] << 8) | b[0]);
  y = (int16_t)((b[3] << 8) | b[2]);
  z = (int16_t)((b[5] << 8) | b[4]);
  return true;
}

static void updateTiltFromAccel() {
  int16_t ax, ay, az;
  if (!adxl345ReadRaw(ax, ay, az)) return;

  float fax = (float)ax;
  float fay = (float)ay;
  float faz = (float)az;

  float roll  = atan2f(fay, faz);
  float pitch = atan2f(-fax, sqrtf(fay*fay + faz*faz));

  int pr = (int)lroundf(pitch * 180.0f / (float)M_PI);
  int rr = (int)lroundf(roll  * 180.0f / (float)M_PI);

  pitchDeg = clampi(pr, -90, 90);
  rollDeg  = clampi(rr, -90, 90);
}

static void sendTiltTelemetry() {
  String p = "P" + String(pitchDeg) + "R" + String(rollDeg);
  sendFrame(12, p);
}

/* ===================== SETUP / LOOP ===================== */

static uint32_t tTilt = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);

  Wire.begin(I2C_SDA, I2C_SCL);

  // Servo timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Gimbals
  g1.setPeriodHertz(50);
  g2.setPeriodHertz(50);
  g3.setPeriodHertz(50);
  g1.attach(PIN_GIMBAL1, SERVO_MIN_US, SERVO_MAX_US);
  g2.attach(PIN_GIMBAL2, SERVO_MIN_US, SERVO_MAX_US);
  g3.attach(PIN_GIMBAL3, SERVO_MIN_US, SERVO_MAX_US);

  // Suspension
  suspF.setPeriodHertz(50);
  suspB.setPeriodHertz(50);
  suspF.attach(PIN_SUSP_F, SERVO_MIN_US, SERVO_MAX_US);
  suspB.attach(PIN_SUSP_B, SERVO_MIN_US, SERVO_MAX_US);

  // Safe initial state
  crServoWriteSpeed(g1, 0);
  crServoWriteSpeed(g2, 0);
  crServoWriteSpeed(g3, 0);
  crServoWriteSpeed(suspF, 0);
  crServoWriteSpeed(suspB, 0);

  motorsInit();

  pinMode(PIN_ENC1, INPUT);
  pinMode(PIN_ENC2, INPUT);
  analogReadResolution(12);

  bool ok = adxl345Init();
  Serial.printf("[IMU] ADXL345 init: %s (addr 0x%02X)\n", ok ? "OK" : "FAIL", ADXL345_ADDR);

  nirOk = nirInit();
  Serial.printf("[NIR] AS7265x init: %s\n", nirOk ? "OK" : "FAIL");

  if(!ina226.init()){
    Serial.println("[PWR] INA226 FAIL");
  }
  ina226.setResistorRange(0.004, 20.0);
  
  uint32_t now = millis();
  tTilt = now;
  last_energy_update = now;

  Serial.println("RooLowLevelESP32 ready");
}

void loop() {
  parseSerial();

  uint32_t now = millis();

  // 1. Update Energy Accumulation (Polled at 50Hz)
  if (now - last_energy_update >= POWER_PERIOD_MS) {
    float delta_h = (now - last_energy_update) / 3600000.0;
    last_energy_update = now;

    current_ma = ina226.getCurrent_mA();
    bus_v = ina226.getBusVoltage_V();
    if (is_recording_energy) {
      accumulated_energy_mwh += (bus_v * current_ma * delta_h);
    }
  }

  // 2. Telemetry Burst @ 4 Hz
  if (now - tTilt >= TILT_PERIOD_MS) {
    tTilt = now;
    
    updateTiltFromAccel();
    sendTiltTelemetry();

    Serial.print("$14~");
    Serial.print(bus_v); Serial.print(",");
    Serial.print(current_ma); Serial.print(",");
    Serial.print(accumulated_energy_mwh);
    Serial.println("`");

    int raw1 = analogRead(PIN_ENC1);
    int raw2 = analogRead(PIN_ENC2);
    
    enc1_deg = (raw1 / 4095.0) * 360.0;
    enc2_deg = (raw2 / 4095.0) * 360.0;

    Serial.print("$15~");
    Serial.print(enc1_deg, 1); Serial.print(",");
    Serial.print(enc2_deg, 1);
    Serial.println("`");
  }
}
