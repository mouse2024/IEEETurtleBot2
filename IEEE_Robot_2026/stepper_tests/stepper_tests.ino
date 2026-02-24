#include <Arduino.h>
#include <Servo.h>

// Stepper Motor
#define ENA_1 2
#define STEP_1 4
#define DIR_1 3

// Servos
Servo leftServo;
Servo rightServo;
int leftPos = 0;
int rightPos = 0;

void setup() {
  pinMode(ENA_1, OUTPUT);
  pinMode(STEP_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);

  digitalWrite(ENA_1, LOW);
  leftServo.attach(8);
  rightServo.attach(9);
}

void loop() {
  motorstep(200, 1);  // Up
  turnServos(0);
  motorstep(200, 0);  //Down
  turnServos(1);
}

void motorstep(int numSteps, int direction) {
  if (direction == 0) {
    digitalWrite(DIR_1, LOW);
  } else {
    digitalWrite(DIR_1, HIGH);
  }
  for (int x = 0; x < numSteps; x++) {
    digitalWrite(STEP_1, HIGH);
    delay(1);  // by changing this time delay between the steps we can change the rotation speed
    digitalWrite(STEP_1, LOW);
    delay(1);
  }
}

void turnServos(int direction) {
  if (direction == 0) {
    rightPos = 180;
    for (leftPos = 0; leftPos < 180; leftPos++) {
      leftServo.write(leftPos);
      rightServo.write(rightPos);
      rightPos--;
      delay(15);
    }
  } else {
    leftPos = 180;
    for (rightPos = 0; rightPos < 180; rightPos++) {
      leftServo.write(leftPos);
      rightServo.write(rightPos);
      leftPos--;
      delay(15);
    }
  }
}