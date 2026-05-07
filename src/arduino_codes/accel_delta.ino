#include <AccelStepper.h>

#define EN_PIN 2

AccelStepper stepper0(AccelStepper::DRIVER, 3, 4);
AccelStepper stepper1(AccelStepper::DRIVER, 6, 7);
AccelStepper stepper2(AccelStepper::DRIVER, 9, 10);

AccelStepper* steppers[3] = { &stepper0, &stepper1, &stepper2 };

// ── Flip true/false per motor if it runs backwards ───────
const bool INVERT_DIR[3] = { true, false, true };

const float MAX_SPEED = 8000.0;
const float MAX_ACCEL = 6000.0;

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  for (int i = 0; i < 3; i++) {
    steppers[i]->setMaxSpeed(MAX_SPEED);
    steppers[i]->setAcceleration(MAX_ACCEL);
    steppers[i]->setCurrentPosition(0);
    steppers[i]->setPinsInverted(INVERT_DIR[i], false, false);  // dir, step, enable
  }

  Serial.println("READY");
}

void loop() {
  readSerial();
  for (int i = 0; i < 3; i++) {
    steppers[i]->run();
  }
}

// ── Parse "p1 p2 p3\n" ───────────────────────────────────
void readSerial() {
  static String buf = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      long values[3] = { 0, 0, 0 };
      int idx = 0;
      String token = "";

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

    } else {
      buf += c;
    }
  }
}