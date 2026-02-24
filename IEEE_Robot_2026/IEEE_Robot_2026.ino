#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// Relay for Electromagnet
#define RELAY_PIN 7

//Stepper Motors
#define ENA_1 2
#define DIR_1 3
#define STEP_1 4

// Servos
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50

void setup() {
  Serial.begin(115200);  // USB serial to Pi

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off

  // Setup Stepper Motors
  pinMode(ENA_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(STEP_1, OUTPUT);
  digitalWrite(ENA_1, LOW);

  servos.begin();
  servos.setPWMFreq(SERVO_FREQ);
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
  if (direction == 0) {
    for (int pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      servos.setPWM(1, 0, SERVOMAX - pulselen);
      servos.setPWM(0, 0, pulselen);
      delay(15);
    }
  } else if (direction == 1) {
    for (int pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      servos.setPWM(1, 0, pulselen);
      servos.setPWM(0, 0, SERVOMAX - pulselen);
      delay(15);
    }
  }
}