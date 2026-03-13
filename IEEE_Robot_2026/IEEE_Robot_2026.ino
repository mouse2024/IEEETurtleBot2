#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// Relay for Electromagnet
#define RELAY_PIN_L 8
#define RELAY_PIN_R 9

//Stepper Motors
#define ENA_1 7
#define DIR_1 3
#define STEP_1 4

#define LIMIT_SWITCH 2
bool limitTrigger = false;
#define DEBOUNCE_DELAY 100
unsigned long lastDebounceTime = 0;

// Servos
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
#define SERVOMIN 85  // about 0 degrees
#define SERVOMAX 525  // about 180 degrees
#define SERVO_FREQ 50

// Start Light Sensor
#define PHOTOCELL_F A0
#define PHOTOCELL_B A1
#define LED 12
int frontReading;
int backReading;
bool start = false;

// Linear Actuator
#define A1_IN1 6
#define A1_IN2 8
#define A2_IN1 9
#define A2_IN2 10


void setup() {
  Serial.begin(115200);  // USB serial to Pi

  // Setup Start LED
  pinMode(PHOTOCELL_F, INPUT);
  pinMode(PHOTOCELL_B, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);


  pinMode(RELAY_PIN_L, OUTPUT);
  pinMode(RELAY_PIN_R, OUTPUT);
  digitalWrite(RELAY_PIN_L, LOW);  // relay off
  digitalWrite(RELAY_PIN_R, LOW);

  // Setup Stepper Motor
  pinMode(ENA_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(STEP_1, OUTPUT);
  digitalWrite(ENA_1, LOW);
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH), switchInterrupt, FALLING);

  // Setup Servos
  servos.begin();
  servos.setPWMFreq(SERVO_FREQ);
  servos.setPWM(0, 0, SERVOMIN);
  servos.setPWM(1, 0, SERVOMAX);
}

void loop() {
  if (!start) {
    start = startLED();
  }
  // if (!limitTrigger) {
  //   Serial.println("frog");
  // }
  if (Serial.available()) {  // command byte + 2 data bytes
    if (Serial.read() == 0xFF) {
      while (Serial.available() < 3)
        ;
      uint8_t command = Serial.read();
      uint8_t data1 = Serial.read();
      uint8_t data2 = Serial.read();

      switch (command) {
        case 0x01:                  // Shovel Stepper
          motorStep(data1, data2);  // 0 = down, 1 = up
          Serial.write(0xAA);
          break;
        case 0x02:
          motorFull(data1);
          Serial.write(0xAA);
          break;
        case 0x03:  // Linear Actuators
          turnServos(data1);
          Serial.write(0xAA);
          break;
        case 0x04:  // Relay
          tiltRobot(data1);
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

  for (int i = 0; i < (numSteps * 5); i++) {
    digitalWrite(STEP_1, HIGH);
    delay(1);
    digitalWrite(STEP_1, LOW);
    delay(1);
  }
}

void motorFull(int direction) {
  if (direction == 0) {
    digitalWrite(DIR_1, LOW);
    while (!limitTrigger) {
      digitalWrite(STEP_1, HIGH);
      delay(1);
      digitalWrite(STEP_1, LOW);
      delay(1);
    }
  } else if (direction == 1) {
    digitalWrite(DIR_1, HIGH);
  }
}

void tiltRobot(int setting) {
  if (setting == 0x00) {
    digitalWrite(RELAY_PIN_L, LOW);  // OFF
    digitalWrite(RELAY_PIN_R, LOW);
  } else if (setting == 0x01) {
    digitalWrite(RELAY_PIN_L, HIGH);  // EXTRACT or RETRACT (not sure yet lol)
    digitalWrite(RELAY_PIN_R, LOW);
  } else if (setting == 0x02) {
    digitalWrite(RELAY_PIN_L, LOW);  // EXTRACT or RETRACT (not sure yet lol)
    digitalWrite(RELAY_PIN_R, HIGH);

  }
}

void turnServos(int direction) {
  if (direction == 0) {
    servos.setPWM(0, 0, SERVOMIN);
    servos.setPWM(1, 0, SERVOMAX);
  } else if (direction == 1) {
    servos.setPWM(0, 0, 375);
    servos.setPWM(1, 0, 375);
  } else if (direction == 2) {
    servos.setPWM(0, 0, SERVOMIN);
    servos.setPWM(1, 0, SERVOMAX);
    delay(500);
    servos.setPWM(0, 0, SERVOMAX);
    servos.setPWM(1, 0, SERVOMIN);
  }
}

void switchInterrupt() {
  if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
    limitTrigger = true;
    lastDebounceTime = millis();
  }
}