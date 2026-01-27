/*
  Roo ESP32 Low-Level Controller (ESP32 DevKit 38-pin, Arduino ESP32 core 3.x)

  Serial protocol: $ID~message`

  Incoming commands (from Pi):
    ID 1: Gimbal Servo 1 angle deg (0..180)
    ID 2: Gimbal Servo 2 angle deg (0..180)
    ID 3: Gimbal Servo 3 angle deg (0..180)
    ID 4: Suspension Front CR servo speed (-10..+10)
    ID 5: Suspension Back  CR servo speed (-10..+10)
    ID 6: Left  motor speed (-10..+10)
    ID 7: Right motor speed (-10..+10)

  Telemetry (to Pi):
    ID 10: INA226 => "V12.34-I0.56"
    ID 11: Encoders => "F180B92"
    ID 12: Tilt => "P12R-3"

  Key change vs earlier versions:
    - Gimbal servo setup + drive uses the exact same working method as your servo test
      (allocateTimer + 50Hz + attach(minUs,maxUs)).
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

/* ===================== CONFIG ===================== */

// Serial
static const uint32_t SERIAL_BAUD = 115200;

// I2C
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// Servos (gimbal)
static const int PIN_GIMBAL1 = 13;
static const int PIN_GIMBAL2 = 12;  // your test used 12
static const int PIN_GIMBAL3 = 14;

// Servos (suspension continuous rotation)
static const int PIN_SUSP_F  = 33;
static const int PIN_SUSP_B  = 32;

// Servo pulse bounds (match your test)
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2400;

// Continuous rotation servo mapping
static const int CR_STOP_US = 1500;
static const int CR_US_PER_SPEED = 35;   // tune for your CR servos

// Motors (IBT-4)
static const int PIN_MOT_L_IN1 = 17;
static const int PIN_MOT_L_IN2 = 16;
static const int PIN_MOT_R_IN1 = 19;
static const int PIN_MOT_R_IN2 = 18;

static const int PWM_FREQ = 20000;
static const int PWM_RES_BITS = 10;
static const int PWM_MAX = (1 << PWM_RES_BITS) - 1;

// Encoders (MT6701 PWM)
static const int PIN_ENC_F = 39;
static const int PIN_ENC_B = 36;

// Suspension encoder soft limits (deg) — adjust
static float SUSP_F_MIN_DEG = 0.0f;
static float SUSP_F_MAX_DEG = 180.0f;
static float SUSP_B_MIN_DEG = 0.0f;
static float SUSP_B_MAX_DEG = 180.0f;

// Telemetry rates
static const uint32_t INA_PERIOD_MS  = 500;
static const uint32_t ENC_PERIOD_MS  = 100;
static const uint32_t TILT_PERIOD_MS = 100;

// INA226
static const uint8_t INA226_ADDR = 0x40;
static const float INA_SHUNT_OHMS = 0.01f;  // set to your board

// MPU6050 (GY-85 accel)
static const uint8_t MPU6050_ADDR = 0x68;

/* ===================== ARDUINO PROTOTYPE FIX ===================== */

struct PwmCapture {
  volatile uint32_t lastRiseUs = 0;
  volatile uint32_t periodUs   = 0;
  volatile uint32_t highUs     = 0;
  volatile bool     lastHigh   = false;
};

static float readEncoderDeg(PwmCapture &cap);

/* ===================== HELPERS ===================== */

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}
static inline int clampi(int x, int lo, int hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static void sendFrame(uint8_t id, const String& payload) {
  Serial.print('$');
  Serial.print(id);
  Serial.print('~');
  Serial.print(payload);
  Serial.print('`');
}

static bool isSignedInt(const String& s) {
  if (s.length() == 0) return false;
  int i = 0;
  if (s[0] == '+' || s[0] == '-') {
    if (s.length() == 1) return false;
    i = 1;
  }
  for (; i < s.length(); i++) {
    char c = s[i];
    if (c < '0' || c > '9') return false;
  }
  return true;
}

/* ===================== ENCODER CAPTURE ===================== */

static PwmCapture encF, encB;
static float encFrontDeg = NAN;
static float encBackDeg  = NAN;

void IRAM_ATTR isrEncF() {
  bool level = digitalRead(PIN_ENC_F);
  uint32_t now = micros();
  if (level) {
    encF.periodUs = now - encF.lastRiseUs;
    encF.lastRiseUs = now;
    encF.lastHigh = true;
  } else {
    if (encF.lastHigh) encF.highUs = now - encF.lastRiseUs;
    encF.lastHigh = false;
  }
}

void IRAM_ATTR isrEncB() {
  bool level = digitalRead(PIN_ENC_B);
  uint32_t now = micros();
  if (level) {
    encB.periodUs = now - encB.lastRiseUs;
    encB.lastRiseUs = now;
    encB.lastHigh = true;
  } else {
    if (encB.lastHigh) encB.highUs = now - encB.lastRiseUs;
    encB.lastHigh = false;
  }
}

static float ENCODER_ANGLE_FROM_DUTY(float duty01) {
  return duty01 * 360.0f;
}

static float readEncoderDeg(PwmCapture &cap) {
  uint32_t p, h;
  noInterrupts();
  p = cap.periodUs;
  h = cap.highUs;
  interrupts();

  if (p == 0 || h == 0 || h > p) return NAN;
  float duty = (float)h / (float)p;
  duty = clampf(duty, 0.0f, 1.0f);

  float deg = ENCODER_ANGLE_FROM_DUTY(duty);
  while (deg < 0) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

/* ===================== LEDC (ESP32 core 3.x) ===================== */

static int chL1 = -1, chL2 = -1, chR1 = -1, chR2 = -1;

static void ledcInitMotorPwm() {
  chL1 = ledcAttach(PIN_MOT_L_IN1, PWM_FREQ, PWM_RES_BITS);
  chL2 = ledcAttach(PIN_MOT_L_IN2, PWM_FREQ, PWM_RES_BITS);
  chR1 = ledcAttach(PIN_MOT_R_IN1, PWM_FREQ, PWM_RES_BITS);
  chR2 = ledcAttach(PIN_MOT_R_IN2, PWM_FREQ, PWM_RES_BITS);

  if (chL1 < 0 || chL2 < 0 || chR1 < 0 || chR2 < 0) {
    Serial.println("ERROR: LEDC attach failed on one or more motor pins.");
    return;
  }

  ledcWrite(chL1, 0); ledcWrite(chL2, 0);
  ledcWrite(chR1, 0); ledcWrite(chR2, 0);
}

static void motorWrite(int chFwd, int chRev, int sp10) {
  sp10 = clampi(sp10, -10, 10);
  int duty = (abs(sp10) * PWM_MAX) / 10;

  if (sp10 > 0) { ledcWrite(chFwd, duty); ledcWrite(chRev, 0); }
  else if (sp10 < 0) { ledcWrite(chFwd, 0); ledcWrite(chRev, duty); }
  else { ledcWrite(chFwd, 0); ledcWrite(chRev, 0); }
}

/* ===================== INA226 ===================== */

static bool i2cRead16(uint8_t addr, uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  out = ((uint16_t)msb << 8) | lsb;
  return true;
}

static bool i2cWrite16(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission(true) == 0;
}

static void ina226Init() {
  i2cWrite16(INA226_ADDR, 0x00, 0x4127);
}

static bool ina226Read(float &volts, float &amps) {
  uint16_t busRaw = 0, shuntRawU16 = 0;
  if (!i2cRead16(INA226_ADDR, 0x02, busRaw)) return false;
  if (!i2cRead16(INA226_ADDR, 0x01, shuntRawU16)) return false;

  volts = (float)busRaw * 1.25e-3f;
  int16_t shuntRaw = (int16_t)shuntRawU16;
  float vshunt = (float)shuntRaw * 2.5e-6f;
  amps = vshunt / INA_SHUNT_OHMS;
  return true;
}

/* ===================== MPU6050 (tilt) ===================== */

static void mpu6050Init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

static bool mpu6050ReadAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU6050_ADDR, 6) != 6) return false;

  ax = ((int16_t)Wire.read() << 8) | Wire.read();
  ay = ((int16_t)Wire.read() << 8) | Wire.read();
  az = ((int16_t)Wire.read() << 8) | Wire.read();
  return true;
}

static bool readPitchRollDeg(float &pitchDeg, float &rollDeg) {
  int16_t ax, ay, az;
  if (!mpu6050ReadAccel(ax, ay, az)) return false;

  float fax = (float)ax, fay = (float)ay, faz = (float)az;
  rollDeg  = atan2f(fay, faz) * 180.0f / PI;
  pitchDeg = atan2f(-fax, sqrtf(fay*fay + faz*faz)) * 180.0f / PI;
  return true;
}

/* ===================== SERVOS ===================== */

static Servo g1, g2, g3;
static Servo suspF, suspB;

// Desired + applied gimbal angles.
// We apply them at 50Hz (20ms) to mimic your test behavior and avoid jitter from “spam”.
static volatile int g1_target = 90;
static volatile int g2_target = 90;
static volatile int g3_target = 90;
static int g1_applied = -1;
static int g2_applied = -1;
static int g3_applied = -1;

static uint32_t lastGimbalApplyMs = 0;
static const uint32_t GIMBAL_APPLY_PERIOD_MS = 20; // 50 Hz

static void applyGimbalsIfDue() {
  uint32_t now = millis();
  if (now - lastGimbalApplyMs < GIMBAL_APPLY_PERIOD_MS) return;
  lastGimbalApplyMs = now;

  int a1 = clampi((int)g1_target, 0, 180);
  int a2 = clampi((int)g2_target, 0, 180);
  int a3 = clampi((int)g3_target, 0, 180);

  if (a1 != g1_applied) { g1.write(a1); g1_applied = a1; }
  if (a2 != g2_applied) { g2.write(a2); g2_applied = a2; }
  if (a3 != g3_applied) { g3.write(a3); g3_applied = a3; }
  if (a2 != g2_applied) {
    g2.write(a2);
    g2_applied = a2;
    Serial.printf("[APPLY] G2=%d\n", a2);
  }

}

static void crServoWriteSpeed(Servo &s, int speed10, float encDeg, float minDeg, float maxDeg) {
  speed10 = clampi(speed10, -10, 10);

  if (!isnan(encDeg)) {
    if (encDeg >= maxDeg && speed10 > 0) speed10 = 0;
    if (encDeg <= minDeg && speed10 < 0) speed10 = 0;
  }

  int us = CR_STOP_US + speed10 * CR_US_PER_SPEED;
  us = clampi(us, SERVO_MIN_US, SERVO_MAX_US);
  s.writeMicroseconds(us);
}

/* ===================== SERIAL PARSER ===================== */

static String rxBuf;

static void handleFrame(uint8_t id, const String &payload) {
  String p = payload;
  p.trim(); // <-- allow "90 ", " 90", "\r\n"

  // Strict numeric-only for command IDs
  if (id >= 1 && id <= 7) {
    if (!isSignedInt(p)) return;
  }

  int val = p.toInt();

  switch (id) {
    case 1: {
      int nv = clampi(val, 0, 180);
      if (nv != g1_target) {
        g1_target = nv;
        Serial.printf("[RX] G1 target=%d\n", nv);
      }
    } break;

    case 2: {
      int nv = clampi(val, 0, 180);
      if (nv != g2_target) {
        g2_target = nv;
        Serial.printf("[RX] G2 target=%d\n", nv);
      }
    } break;

    case 3: {
      int nv = clampi(val, 0, 180);
      if (nv != g3_target) {
        g3_target = nv;
        Serial.printf("[RX] G3 target=%d\n", nv);
      }
    } break;

    case 4: crServoWriteSpeed(suspF, val, encFrontDeg, SUSP_F_MIN_DEG, SUSP_F_MAX_DEG); break;
    case 5: crServoWriteSpeed(suspB, val, encBackDeg,  SUSP_B_MIN_DEG, SUSP_B_MAX_DEG); break;

    case 6:
      if (chL1 >= 0 && chL2 >= 0) motorWrite(chL1, chL2, val);
      break;

    case 7:
      if (chR1 >= 0 && chR2 >= 0) motorWrite(chR1, chR2, val);
      break;

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

    if (rxBuf.length() > 256) rxBuf = ""; // drop garbage
  }
}


/* ===================== TELEMETRY ===================== */

static uint32_t tIna = 0, tEnc = 0, tTilt = 0;

static void sendInaTelemetry() {
  float v, a;
  if (!ina226Read(v, a)) return;
  String p = "V" + String(v, 2) + "-I" + String(a, 2);
  sendFrame(10, p);
}

static void sendEncTelemetry() {
  int f = isnan(encFrontDeg) ? 0 : (int)lroundf(encFrontDeg);
  int b = isnan(encBackDeg)  ? 0 : (int)lroundf(encBackDeg);
  String p = "F" + String(f) + "B" + String(b);
  sendFrame(11, p);
}

static void sendTiltTelemetry() {
  float pitch, roll;
  if (!readPitchRollDeg(pitch, roll)) return;
  String p = "P" + String((int)lroundf(pitch)) + "R" + String((int)lroundf(roll));
  sendFrame(12, p);
}

/* ===================== SETUP / LOOP ===================== */

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);

  // ---- EXACTLY like your working servo test ----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // I2C + sensors
  Wire.begin(I2C_SDA, I2C_SCL);
  ina226Init();
  mpu6050Init();

  // ---- GIMBAL: match your test style ----
  g1.setPeriodHertz(50);
  g2.setPeriodHertz(50);
  g3.setPeriodHertz(50);

  g1.attach(PIN_GIMBAL1, SERVO_MIN_US, SERVO_MAX_US);
  g2.attach(PIN_GIMBAL2, SERVO_MIN_US, SERVO_MAX_US);
  g3.attach(PIN_GIMBAL3, SERVO_MIN_US, SERVO_MAX_US);

  // ---- Suspension servos (CR) ----
  suspF.setPeriodHertz(50);
  suspB.setPeriodHertz(50);
  suspF.attach(PIN_SUSP_F, SERVO_MIN_US, SERVO_MAX_US);
  suspB.attach(PIN_SUSP_B, SERVO_MIN_US, SERVO_MAX_US);

  // Safe initial outputs
  g1_target = 90; g2_target = 90; g3_target = 90;
  g1.write(90); g2.write(90); g3.write(90);
  suspF.writeMicroseconds(CR_STOP_US);
  suspB.writeMicroseconds(CR_STOP_US);
  delay(200);

  // Motors PWM
  ledcInitMotorPwm();

  // Encoders
  pinMode(PIN_ENC_F, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_F), isrEncF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncB, CHANGE);

  uint32_t now = millis();
  tIna = tEnc = tTilt = now;
  lastGimbalApplyMs = now;

  Serial.println("RooLowLevelESP32 ready");
}

void loop() {
  parseSerial();

  // Apply gimbal angles at 50Hz (just like your test loop delay(20))
  applyGimbalsIfDue();

  // Update encoder angles
  float f = readEncoderDeg(encF);
  float b = readEncoderDeg(encB);
  if (!isnan(f)) encFrontDeg = f;
  if (!isnan(b)) encBackDeg  = b;

  uint32_t now = millis();
  if (now - tIna  >= INA_PERIOD_MS)  { tIna  = now; sendInaTelemetry(); }
  if (now - tEnc  >= ENC_PERIOD_MS)  { tEnc  = now; sendEncTelemetry(); }
  if (now - tTilt >= TILT_PERIOD_MS) { tTilt = now; sendTiltTelemetry(); }
}
