#include <Servo.h>

Servo leftMotor;
Servo rightMotor;

const int MIN_US  = 1000;
const int STOP_US = 1500;
const int MAX_US  = 2000;

// reverseRight = true if your right motor needs reversing
const bool reverseRight = true;

// Clamp helper
int clampUS(int us) {
  if (us < MIN_US) return MIN_US;
  if (us > MAX_US) return MAX_US;
  return us;
}

// Set both motors "simultaneously" (same loop tick)
void setMotorsUS(int leftUS, int rightUS) {
  leftUS  = clampUS(leftUS);
  rightUS = clampUS(rightUS);

  if (reverseRight) {
    rightUS = 3000 - rightUS;   // mirror around 1500
  }

  // Write back-to-back each control cycle
  leftMotor.writeMicroseconds(leftUS);
  rightMotor.writeMicroseconds(rightUS);
}

void armControllers() {
  // Many ESCs/controllers want neutral for a bit before responding
  setMotorsUS(STOP_US, STOP_US);
  delay(2000);

  // Optional: a couple extra neutral writes for stubborn controllers
  for (int i = 0; i < 20; i++) {
    setMotorsUS(STOP_US, STOP_US);
    delay(20);
  }
}

void setup() {
  leftMotor.attach(2, MIN_US, MAX_US);
  rightMotor.attach(4, MIN_US, MAX_US);

  armControllers();
}

void loop() {
  // Example: forward
  setMotorsUS(1700, 1700);
  delay(2000);

  // stop
  setMotorsUS(STOP_US, STOP_US);
  delay(1000);

  // reverse
  setMotorsUS(1300, 1300);
  delay(2000);

  setMotorsUS(STOP_US, STOP_US);
  delay(1000);
}