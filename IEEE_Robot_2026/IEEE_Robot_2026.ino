#include <Arduino.h>
#include <Servo.h>

// Relay for Electromagnet
#define RELAY_PIN 7

//Stepper Motors
#define ENA_1 2
#define DIR_1 3
#define STEP_1 4

// Servos
Servo leftServo;
Servo rightServo;
int servoPos = 0;

void setup() {
  Serial.begin(115200);  // USB serial to Pi

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off

  // Setup Stepper Motors
  pinMode(ENA_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(STEP_1, OUTPUT);
  digitalWrite(ENA_1, LOW);

  leftServo.write(90);  // Prevent servo from jumping (hopefully)
  rightServo.write(90);
  delay(50);
  leftServo.attach(8);
  rightServo.attach(9);
}

void loop() {
  if (Serial.available()) {  // command byte + 2 data bytes
    if (Serial.read() == 0xFF) {
      while (Serial.available() < 3)
        ;
      uint8_t cmd = Serial.read();
      uint8_t data1 = Serial.read();
      uint8_t data2 = Serial.read();

      switch (cmd) {
        case 0x01:                  // Shovel Stepper
          motorStep(data1, data2);  // 0 = down, 1 = up
          Serial.write(0xAA);
          break;
        case 0x02:  // Relay
          setRelay(data1);
          Serial.write(0xAA);
          break;
        case 0x03:  // Servos
          turnServos(data1);
          Serial.write(0xAA);
          break;
        default:
          Serial.write(0xFF);  // ERROR: unknown command
          break;
      }
    }
  }
}

void motorStep(int numSteps, int direction) {
  if (direction == 0) {  // move down
    digitalWrite(DIR_1, LOW);
  } else if (direction == 1) {  // move up
    digitalWrite(DIR_1, HIGH);
  }

  for (int i = 0; i < numSteps; i++) {
    digitalWrite(STEP_1, HIGH);
    delay(1);
    digitalWrite(STEP_1, LOW);
    delay(1);
  }
}

void setRelay(int setting) {
  if (setting == 0x00) {
    digitalWrite(RELAY_PIN, LOW);  // OFF
  } else if (setting == 0x01) {
    digitalWrite(RELAY_PIN, HIGH);  // ON
  }
}

void turnServos(int direction) {
  for (servoPos = 0; servoPos < 180; servoPos++) {
    if (direction == 0) {
      leftServo.write(servoPos);
      rightServo.write(180 - servoPos);
    } else if (direction == 1) {
      leftServo.write(180 - servoPos);
      rightServo.write(servoPos);
    }
    delay(15);
  }
}