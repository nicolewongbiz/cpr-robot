#include <Arduino.h>

// ---------------- PIN DEFINITIONS ----------------
// Motor A (Left)
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// Motor B (Right)
const int enB = 3;
const int in3 = 5;
const int in4 = 4;

// ---------------- TUNING ----------------
// If your robot goes the wrong way, flip one of these:
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;

// Convert command units -> PWM. Tune these.
const float VEL_TO_PWM = 220.0f;   // bigger = faster for same cmd_vel
const int   PWM_MAX    = 255;
const int   PWM_MIN    = 0;
const int   PWM_DEADBAND = 20;     // below this, treat as 0 (overcomes stiction)

// Safety: stop if no cmd_vel for this long
const unsigned long CMD_TIMEOUT_MS = 400;

// Debug prints back over serial
const bool DEBUG = true;

String line;
unsigned long last_cmd_ms = 0;

void stopMotors() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int pwmFromFloat(float u) {
  // u is roughly in [-1,1] if you publish sane values; but we don't assume.
  int pwm = (int)roundf(fabsf(u) * VEL_TO_PWM);
  pwm = clampInt(pwm, PWM_MIN, PWM_MAX);
  if (pwm < PWM_DEADBAND) pwm = 0;
  return pwm;
}

void setMotorLeft(float u) {
  if (INVERT_LEFT) u = -u;
  int pwm = pwmFromFloat(u);

  if (pwm == 0) {
    // Coast (no braking)
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
    return;
  }

  if (u >= 0) { // forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {      // backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA, pwm);
}

void setMotorRight(float u) {
  if (INVERT_RIGHT) u = -u;
  int pwm = pwmFromFloat(u);

  if (pwm == 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);
    return;
  }

  if (u >= 0) { // forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {      // backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  analogWrite(enB, pwm);
}

void applyCmdVel(float v, float w) {
  // Differential drive mix
  float left  = v - w;
  float right = v + w;

  setMotorLeft(left);
  setMotorRight(right);

  if (DEBUG) {
    Serial.print("ACK VEL v="); Serial.print(v, 3);
    Serial.print(" w="); Serial.print(w, 3);
    Serial.print(" L="); Serial.print(left, 3);
    Serial.print(" R="); Serial.println(right, 3);
  }
}

void setup() {
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  stopMotors();

  Serial.begin(115200);
  while (!Serial) {}
  last_cmd_ms = millis();

  if (DEBUG) Serial.println("READY");
}

void loop() {
  // Watchdog safety stop
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line.trim();
      if (line.length() == 0) { line = ""; continue; }

      // Parse commands:
      // "VEL <v> <w>" or "STOP"
      if (line.startsWith("STOP")) {
        stopMotors();
        if (DEBUG) Serial.println("DONE STOP");
      } else if (line.startsWith("VEL")) {
        float v = 0.0f, w = 0.0f;

        // Very simple parsing: split by spaces
        int s1 = line.indexOf(' ');
        int s2 = (s1 >= 0) ? line.indexOf(' ', s1 + 1) : -1;
        if (s1 > 0 && s2 > s1) {
          v = line.substring(s1 + 1, s2).toFloat();
          w = line.substring(s2 + 1).toFloat();
          last_cmd_ms = millis();
          applyCmdVel(v, w);
        } else {
          if (DEBUG) Serial.println("ERR VEL format: VEL <v> <w>");
        }
      } else {
        if (DEBUG) { Serial.print("ERR UNKNOWN CMD: "); Serial.println(line); }
      }

      line = "";
    } else {
      if (line.length() < 80) line += c;
    }
  }
}
