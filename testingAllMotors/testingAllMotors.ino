#include <Arduino.h>

// ----- Motor Driver 1 (LEFT side) -----
const int ENA1  = 3;   // PWM
const int IN1_1 = 2;
const int IN2_1 = 4;

const int IN3_1 = 5;
const int IN4_1 = 7;
const int ENB1  = 6;   // PWM

// ----- Motor Driver 2 (RIGHT side) -----
const int ENA2  = 9;   // PWM
const int IN1_2 = 8;
const int IN2_2 = 11;

const int IN3_2 = 12;
const int IN4_2 = 13;
const int ENB2  = 10;

const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;

// PWM tuning
const int PWM_MAX = 255;
const int PWM_DEADBAND_DEFAULT = 0;

// Base mapping: |cmd| * CMD_TO_PWM -> PWM
const float CMD_TO_PWM = 220.0f;

// ---------- Gains (trim) ----------
const float GAIN_ENA1 = 0.80f;
const float GAIN_ENB1 = 0.80f;
const float GAIN_ENA2 = 0.80f;
const float GAIN_ENB2 = 0.80f;

// ---------- Start PWM ----------
const int START_ENA1 = 25;
const int START_ENB1 = 25;
const int START_ENA2 = 25;
const int START_ENB2 = 25;

// ---------- Scale ----------
const float SCALE_ENA1 = 1.00f;
const float SCALE_ENB1 = 1.20f;
const float SCALE_ENA2 = 1.00f;
const float SCALE_ENB2 = 1.00f;

// --------- Straight-line trim ---------
// + trims to the RIGHT (boost right, reduce left)
// - trims to the LEFT  (boost left,  reduce right)
float STRAIGHT_TRIM = 0.00f;     // try 0.02, -0.02, etc.
// -------------------------------------

// Smooth motion tuning
const float MAX_V_ACCEL = 1.2f;
const float MAX_V_DECEL = 2.0f;
const float MAX_W_ACCEL = 2.0f;
const float MAX_W_DECEL = 3.0f;

const unsigned long CMD_TIMEOUT_MS = 600;
const unsigned long DEMO_RUN_MS = 2000;

unsigned long last_cmd_ms = 0;

// Targets
float v_target = 0.0f, w_target = 0.0f;
// Smoothed/current
float v_now = 0.0f, w_now = 0.0f;

unsigned long start_ms = 0;
bool stopped = false;

// ---------- helpers ----------
int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int basePwmFromCmd(float u, float gain) {
  float ug = u * gain;
  ug = clampFloat(ug, -1.0f, 1.0f);
  int pwm = (int)roundf(fabsf(ug) * CMD_TO_PWM);
  return clampInt(pwm, 0, PWM_MAX);
}

int shapePwm(int pwm_in, int start_pwm, float scale, int deadband) {
  pwm_in = clampInt(pwm_in, 0, PWM_MAX);
  if (pwm_in == 0) return 0;

  pwm_in = max(pwm_in, start_pwm);

  int above = pwm_in - start_pwm;
  int out = start_pwm + (int)roundf(scale * above);

  out = clampInt(out, 0, PWM_MAX);
  if (out < deadband) out = 0;
  return out;
}

float slewToward(float now, float target, float maxUpPerSec, float maxDownPerSec, float dt) {
  float err = target - now;
  if (err == 0.0f) return now;

  bool increasing_mag = fabsf(target) > fabsf(now);
  float maxStep = (increasing_mag ? maxUpPerSec : maxDownPerSec) * dt;

  if (err > 0) {
    if (err > maxStep) err = maxStep;
  } else {
    if (-err > maxStep) err = -maxStep;
  }
  return now + err;
}

void driveChannel(int inA, int inB, int en,
                  float u,
                  float gain,
                  int start_pwm,
                  float scale,
                  int deadband) {
  int base = basePwmFromCmd(u, gain);
  int pwm  = shapePwm(base, start_pwm, scale, deadband);

  if (pwm == 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(en, 0);
    return;
  }

  if (u >= 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }

  analogWrite(en, pwm);
}

void setLeft(float u) {
  if (INVERT_LEFT) u = -u;
  driveChannel(IN1_1, IN2_1, ENA1, u, GAIN_ENA1, START_ENA1, SCALE_ENA1, PWM_DEADBAND_DEFAULT);
  driveChannel(IN3_1, IN4_1, ENB1, u, GAIN_ENB1, START_ENB1, SCALE_ENB1, PWM_DEADBAND_DEFAULT);
}

void setRight(float u) {
  if (INVERT_RIGHT) u = -u;
  driveChannel(IN1_2, IN2_2, ENA2, u, GAIN_ENA2, START_ENA2, SCALE_ENA2, PWM_DEADBAND_DEFAULT);
  driveChannel(IN3_2, IN4_2, ENB2, u, GAIN_ENB2, START_ENB2, SCALE_ENB2, PWM_DEADBAND_DEFAULT);
}

void stopMotors() {
  digitalWrite(IN1_1, LOW); digitalWrite(IN2_1, LOW);
  digitalWrite(IN3_1, LOW); digitalWrite(IN4_1, LOW);
  analogWrite(ENA1, 0); analogWrite(ENB1, 0);

  digitalWrite(IN1_2, LOW); digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, LOW); digitalWrite(IN4_2, LOW);
  analogWrite(ENA2, 0); analogWrite(ENB2, 0);
}

// Straight-line application: apply trim to left/right evenly
void applySmoothedCmd(float v, float w) {
  float left  = v - w;
  float right = v + w;

  // Straight trim: + boosts right, - boosts left
  float t = clampFloat(STRAIGHT_TRIM, -0.30f, 0.30f);
  left  *= (1.0f - t);
  right *= (1.0f + t);

  // Keep within sane range
  left  = clampFloat(left,  -1.0f, 1.0f);
  right = clampFloat(right, -1.0f, 1.0f);

  setLeft(left);
  setRight(right);
}

void updateSmoothing(float dt) {
  v_now = slewToward(v_now, v_target, MAX_V_ACCEL, MAX_V_DECEL, dt);
  w_now = slewToward(w_now, w_target, MAX_W_ACCEL, MAX_W_DECEL, dt);
}

void setup() {
  pinMode(ENA1, OUTPUT);  pinMode(ENB1, OUTPUT);
  pinMode(IN1_1, OUTPUT); pinMode(IN2_1, OUTPUT);
  pinMode(IN3_1, OUTPUT); pinMode(IN4_1, OUTPUT);

  pinMode(ENA2, OUTPUT);  pinMode(ENB2, OUTPUT);
  pinMode(IN1_2, OUTPUT); pinMode(IN2_2, OUTPUT);
  pinMode(IN3_2, OUTPUT); pinMode(IN4_2, OUTPUT);

  stopMotors();

  last_cmd_ms = millis();
  start_ms = last_cmd_ms;
}

void loop() {
  static unsigned long last_ms = millis();
  unsigned long now_ms = millis();
  float dt = (now_ms - last_ms) / 1000.0f;
  last_ms = now_ms;

  if (!stopped) {
    if (now_ms - start_ms < 10000) {
      // drive forward for 2 seconds
      v_target = 0.5f;
      w_target = 0.0f;
      last_cmd_ms = now_ms;
    } else {
      // stop after 2 seconds (and stay stopped)
      v_target = 0.0f;
      w_target = 0.0f;
      updateSmoothing(dt);
      applySmoothedCmd(v_now, w_now);
      stopMotors();
      stopped = true;
      return;
    }
  } else {
    // stay stopped forever
    stopMotors();
    return;
  }

  // safety timeout still works (optional)
  if (now_ms - last_cmd_ms > CMD_TIMEOUT_MS) {
    v_target = 0.0f;
    w_target = 0.0f;
  }

  updateSmoothing(dt);
  applySmoothedCmd(v_now, w_now);
}

