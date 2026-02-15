#include <Stepper.h>
#include <Servo.h>

const int stepsPerRevolution = 200;

// IMPORTANT: avoid pins 0/1 when using Serial
Stepper myStepper(stepsPerRevolution, 5, 2, 3, 4);

const int IR_PIN = 6;
const int SERVO_PIN = 9;
Servo myServo;

const unsigned long LOW_HOLD_MS = 500UL;
const unsigned long SERVO_DELAY_MS = 800UL;

bool servoMode = false;
bool servoSweepEnabled = true;

unsigned long lowStartMs = 0;
unsigned long lastServoToggleMs = 0;
bool servoAt120 = false;

const unsigned long SERVO_RUN_MS = 10000UL;   // servo sweeps for 10s
unsigned long servoStartMs = 0;

bool stepperUpDone = false;                   // step up only once
const unsigned long STEP_UP_MS = 8000UL;     // step up for 15s

bool isBlocked() {
  return (digitalRead(IR_PIN) == LOW); // flip if your module is inverted
}

void setup() {
  pinMode(IR_PIN, INPUT);

  Serial.begin(9600);

  myStepper.setSpeed(50);

  myServo.attach(SERVO_PIN);
  myServo.write(0);
}

void loop() {
  unsigned long now = millis();

  // ---- SEARCH MODE ----
  if (!servoMode) {
    myStepper.step(-200); // searching direction

    if (isBlocked()) {
      if (lowStartMs == 0) lowStartMs = now;
      if (now - lowStartMs >= LOW_HOLD_MS) {
        servoMode = true;
        servoStartMs = now;
        lastServoToggleMs = now;
        servoAt120 = false;
        servoSweepEnabled = true;
        myServo.write(0);
        Serial.println("Blocked 500ms -> start servo sweep");
      }
    } else {
      lowStartMs = 0;
    }
    return;
  }

  // ---- SERVO MODE ----
  if (servoSweepEnabled && (now - lastServoToggleMs >= SERVO_DELAY_MS)) {
    lastServoToggleMs = now;
    servoAt120 = !servoAt120;
    myServo.write(servoAt120 ? 120 : 0);
  }

  // After 10s of servo sweep, stop servo and run stepper UP for 15s (demo-style loop)
  if (!stepperUpDone && (now - servoStartMs >= SERVO_RUN_MS)) {
    servoSweepEnabled = false;
    myServo.write(0);

    Serial.println("Servo done -> stepper UP for 15s");

    unsigned long startMs = millis();
    while (millis() - startMs < STEP_UP_MS) {
      myStepper.step(200);   // UP direction (flip sign if needed)
    }

    Serial.println("Stepper UP done");
    stepperUpDone = true;   // don't do it again
  }
}
