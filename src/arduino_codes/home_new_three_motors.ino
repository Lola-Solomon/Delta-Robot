#include <AccelStepper.h>

#define EN_PIN 15

AccelStepper stepper0(AccelStepper::DRIVER, 2, 3);
AccelStepper stepper1(AccelStepper::DRIVER, 4, 5);
AccelStepper stepper2(AccelStepper::DRIVER, 6, 7);

AccelStepper* steppers[3] = {&stepper0, &stepper1, &stepper2};

const int   LIMIT_PINS[3] = {, 10, 9};
const bool  INVERT_DIR[3] = {true, false, true};
const float MAX_SPEED     = 8000.0;
const float MAX_ACCEL     = 6000.0;
const float HOME_SPEED    = 600.0;

bool joint_homed[3] = {false, false, false};
bool is_homed       = false;
bool started        = false;
static String buf   = "";

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  for (int i = 0; i < 3; i++) {
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
    steppers[i]->setMaxSpeed(HOME_SPEED);
    steppers[i]->setAcceleration(MAX_ACCEL);
    steppers[i]->setCurrentPosition(0);
    steppers[i]->setPinsInverted(INVERT_DIR[i], false, false);
  }

  while (!started) {
    Serial.println("READY");
    unsigned long t = millis();
    while (millis() - t < 500) {
      waitForStart();
      if (started) break;
      delay(10);
    }
  }

  // All motors move toward their limit switches
  for (int i = 0; i < 3; i++) {
    steppers[i]->moveTo(100000);
  }
}

// ─────────────────────────────────────────────────────────
void loop() {
  if (!is_homed) {
    doHoming();
  } else {
    readSerial();
  }

  for (int i = 0; i < 3; i++) steppers[i]->run();
}

// ─────────────────────────────────────────────────────────
void waitForStart() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'S') {
      started = true;
      Serial.flush();
      return;
    }
  }
}

// ─────────────────────────────────────────────────────────
void doHoming() {
  // ── Phase 1: stop each motor independently when its switch triggers ──
  for (int i = 0; i < 3; i++) {
    if (!joint_homed[i] && digitalRead(LIMIT_PINS[i]) == LOW) {
      steppers[i]->setSpeed(0);
      steppers[i]->moveTo(steppers[i]->currentPosition());
      steppers[i]->setCurrentPosition(0);
      joint_homed[i] = true;
    }
  }

  // ── Phase 2: once ALL switches triggered, backoff together ───────────
  bool all_triggered = joint_homed[0] && joint_homed[1] && joint_homed[2];
  if (all_triggered) {
    for (int i = 0; i < 3; i++) {
      steppers[i]->moveTo(-1000);
    }

    // Wait for all to finish backoff
    bool all_done = false;
    while (!all_done) {
      all_done = true;
      for (int i = 0; i < 3; i++) {
        steppers[i]->run();
        if (steppers[i]->distanceToGo() != 0) all_done = false;
      }
    }

    // Set true zero after backoff
    for (int i = 0; i < 3; i++) {
      steppers[i]->setCurrentPosition(0);
      steppers[i]->setMaxSpeed(MAX_SPEED);
    }

    is_homed = true;
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
