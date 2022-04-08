#include <Arduino.h>
#include <ESP32Servo.h>
#ifndef DEFINITIONS_H 
#include <ROAR_Definitions.h>
#endif
#ifndef GLOBALNAMES_H
#include <ROAR_globalnames.h>
#endif

//==========Servo objects===========================================
Servo throttleServo;
Servo steeringServo;

void setupServo() {
  ESP32PWM::timerCount[0]=4;
  ESP32PWM::timerCount[1]=4;
  throttleServo.setPeriodHertz(50);
  throttleServo.attach(THROTTLE_PIN, 1000, 2000);
  steeringServo.setPeriodHertz(50);    // standard 50 hz servo
  steeringServo.attach(STEERING_PIN, 1000, 2000); // attaches the servo on pin (whatever you assign)
}

void checkServo() {
  if (throttleServo.attached() == false) {
    throttleServo.attach(THROTTLE_PIN);
  }
  if (steeringServo.attached() == false) {
    steeringServo.attach(STEERING_PIN);
  }
}

void writeToServo(unsigned int throttle, unsigned int steering) {
  checkServo(); // prevent servo from detaching
  latest_throttle = throttle;
  latest_steering = steering;

  throttleServo.writeMicroseconds(latest_throttle);
  steeringServo.writeMicroseconds(latest_steering);
//  writeToSerial(latest_throttle, latest_steering);
}

void ensureSmoothBackTransition() {
  if (isForwardState and latest_throttle < 1500) {
    writeToServo(1500, latest_steering);
    delay(100);
    writeToServo(1450, latest_steering);
    delay(100);
    writeToServo(1500,latest_steering);
    delay(100);
    isForwardState = false;
  } else if (latest_throttle >= 1500) {
    isForwardState = true;
  }
  return;
}
