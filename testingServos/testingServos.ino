#include <Servo.h>
#include <Stepper.h>

// ---------- Servo (same pins/logic as your code) ----------
Servo myServo;

const int servoPin = 9;   // keep pin 9
const int angleA = 0;
const int angleB = 120;

// ---------- Stepper (same pins/logic as your code) ----------
const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 1, 2, 3, 4);

// ---------- IR sensor ----------
const int irPin = 6;      // IR sensor digital output to pin 8

// Speeds (RPM) — tune as needed
const int stepperSlowRPM = 50;

// Debounce / stability
const unsigned long stableLowMs = 100; // require LOW for this long to trigger servo

void swingServoFiveTimes() {
  for (int i = 0; i < 5; i++) {
    myServo.write(angleB);
    delay(800);
    myServo.write(angleA);
    delay(800);
  }
}

void setup() {
  // Servo setup
  myServo.attach(servoPin);
  myServo.write(angleA);
  delay(1000);

  // IR sensor input
  pinMode(irPin, INPUT);

  // Stepper speed baseline
  myStepper.setSpeed(stepperSlowRPM);
}

void loop() {
  // Assumption per your request:
  // HIGH = high light intensity / no object  -> keep moving anticlockwise
  // LOW  = low light (object present)        -> stop stepper and run servo motion

  int irState = digitalRead(irPin);

  if (irState == HIGH) {
    // Move stepper slowly anti-clockwise (negative steps in your original logic)
    myStepper.setSpeed(stepperSlowRPM);

    // Step in small chunks so we can react quickly if the IR changes
    myStepper.step(-10);
  } else {
    // IR is LOW: stop stepper (we simply don't step it)
    // Optionally wait for the LOW to be stable briefly (helps avoid noise)
    unsigned long t0 = millis();
    while (digitalRead(irPin) == LOW && (millis() - t0) < stableLowMs) {
      // just wait
    }
    if (digitalRead(irPin) == LOW) {
      // Run servo motion 5 times
      swingServoFiveTimes();

      // After servo finishes, wait until IR goes back HIGH
      // so we don't immediately retrigger if the object is still there.
      while (digitalRead(irPin) == LOW) {
        delay(10);
      }

      // Ensure servo returns to angleA (same logic as before)
      myServo.write(angleA);
      delay(200);
    }
  }
}
