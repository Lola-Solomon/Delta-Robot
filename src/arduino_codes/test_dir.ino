#include <AccelStepper.h>

#define EN_PIN 5

// STEP , DIR
AccelStepper m1(AccelStepper::DRIVER, 3, 4);
AccelStepper m2(AccelStepper::DRIVER, 6, 7);
AccelStepper m3(AccelStepper::DRIVER, 9, 10);

int STEPS_180 = 2000;

void setup() {

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);  // enable driver

  m1.setMaxSpeed(8000);
  m1.setAcceleration(6000);

  m2.setMaxSpeed(8000);
  m2.setAcceleration(6000);

  m3.setMaxSpeed(8000);
  m3.setAcceleration(6000);
  // m1->setPinsInverted(false, false, false); // dir, step, enable

  m1.moveTo(-STEPS_180);
  m2.moveTo(STEPS_180);
  m3.moveTo(-STEPS_180);
}

void loop() {





  m1.run();  //neg go up
  m2.run();  //pos go up
  m3.run();  //neg go up
}