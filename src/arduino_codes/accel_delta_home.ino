#include <AccelStepper.h>

#define EN_PIN 2

AccelStepper stepper0(AccelStepper::DRIVER, 3, 4);
AccelStepper stepper1(AccelStepper::DRIVER, 6, 7);
AccelStepper stepper2(AccelStepper::DRIVER, 8, 9);

AccelStepper* steppers[3] = {&stepper0, &stepper1, &stepper2};

// ── Limit switch pins ─────────────────────────────────────
const int LIMIT_PINS[3] = {10, 11, 12};  // change to your pins

const bool  INVERT_DIR[3] = {false, false, false};
const float MAX_SPEED     = 8000.0;
const float MAX_ACCEL     = 6000.0;
const float HOME_SPEED    = 1000.0;

bool joint_homed[3] = {false, false, false};
bool is_homed       = false;

static String buf = "";

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  for (int i = 0; i < 3; i++) {
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);

    steppers[i]->setMaxSpeed(HOME_SPEED);  // slow for homing
    steppers[i]->setAcceleration(MAX_ACCEL);
    steppers[i]->setCurrentPosition(0);
    steppers[i]->setPinsInverted(INVERT_DIR[i], false, false);

    // ── Rotate toward limit switch on startup ─────────────
    steppers[i]->moveTo(-100000);
  }

  Serial.println("HOMING_START");
}

// ─────────────────────────────────────────────────────────
void loop() {
  if (!is_homed) {
    doHoming();          // rotating until all switches pressed
  } else {
    readSerial();        // only reads serial after fully homed
  }

  for (int i = 0; i < 3; i++) steppers[i]->run();
}

// ─────────────────────────────────────────────────────────
void doHoming() {
  bool all_done = true;

  for (int i = 0; i < 3; i++) {
    if (joint_homed[i]) continue;

    all_done = false;

    if (digitalRead(LIMIT_PINS[i]) == LOW) {  // switch pressed
      steppers[i]->stop();
      steppers[i]->setCurrentPosition(0);
      steppers[i]->moveTo(0);
      joint_homed[i] = true;

      Serial.print("JOINT_");
      Serial.print(i + 1);
      Serial.println("_HOMED");
    }
  }

  if (all_done) {
    is_homed = true;

    // ── Unlock full speed after homing ───────────────────
    for (int i = 0; i < 3; i++) {
      steppers[i]->setMaxSpeed(MAX_SPEED);
    }

    Serial.println("HOMED");
  }
}

// ─────────────────────────────────────────────────────────
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    for (int i = 0; i < 3; i++) steppers[i]->run();

    if (c == '\n') {
      long values[3] = {0, 0, 0};
      int  idx       = 0;
      String token   = "";

      for (int i = 0; i <= (int)buf.length() && idx < 3; i++) {
        if (i == (int)buf.length() || buf[i] == ' ') {
          if (token.length() > 0) {
            values[idx++] = token.toInt();
            token = "";
          }
        } else {
          token += buf[i];
        }
      }

      if (idx == 3) {
        for (int j = 0; j < 3; j++) {
          steppers[j]->moveTo(values[j]);
        }
      }
      buf = "";

    } else if (c != '\r') {
      buf += c;
    }
  }
}