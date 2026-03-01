#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// Relay for Electromagnet
#define RELAY_PIN 7

//Stepper Motors
#define ENA_1 2
#define DIR_1 3
#define STEP_1 4

#define LIMIT_SWTICH 9

// Servos
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
#define SERVOMIN 100  // about 0 degrees
#define SERVOMAX 500  // about 180 degrees
#define SERVO_FREQ 50

// Start Light Sensor
#define PHOTOCELL_F A0
#define PHOTOCELL_B A1
#define LED 12
int frontReading;
int backReading;
bool start = false;

void setup() {
  Serial.begin(115200);  // USB serial to Pi

  // Setup Start LED
  pinMode(PHOTOCELL_F, INPUT);
  pinMode(PHOTOCELL_B, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off

  // Setup Stepper Motor
  pinMode(ENA_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(STEP_1, OUTPUT);
  digitalWrite(ENA_1, LOW);
  pinMode(LIMIT_SWITCH, INPUT);

  // Setup Servos
  servos.begin();
  servos.setPWMFreq(SERVO_FREQ);
  servos.setPWM(0, 0, SERVOMAX);
  servos.setPWM(1, 0, SERVOMIN);
}

void loop() {
  if (!start) {
    start = startLED();
  }
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
        case 0x02:
          motorFull(data1);
          Serial.write(0xAA);
          break;
        case 0x03:  // Servos
          turnServos(data1);
          Serial.write(0xAA);
          break;
        case 0x04:  // Relay
          setRelay(data1);
          Serial.write(0xAA);
          break;
        default:
          Serial.write(0xFF);  // ERROR: unknown command
          break;
      }
    }
  }
}

bool startLED() {
  bool trigger = false;
  frontReading = analogRead(PHOTOCELL_F);
  backReading = analogRead(PHOTOCELL_B);
  if (backReading - frontReading >= 200) {
    trigger = true;
    digitalWrite(LED, LOW);
  }
  return trigger;
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

void motorFull(int direction) {
  if (direction = 0) {
    digitalWrite(DIR_1, LOW);
    while (digitalRead(LIMIT_SWITCH) == LOW) {
      digitalWrite(STEP_1, HIGH);
      delay(1);
      digitalWrite(STEP_1, LOW);
      delay(1);
    }
  } else if (direction == 1) {
    digitalWrite(DIR_1, HIGH);
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
    servos.setPWM(0, 0, SERVOMIN);
    servos.setPWM(1, 0, SERVOMAX);
  } else if (direction == 1) {
    servos.setPWM(0, 0, SERVOMAX);
    servos.setPWM(1, 0, SERVOMIN);
  }
}