#include <Arduino.h>
#define IR 25




int check_IR() {
  int reading = analogRead(IR);

  Serial.println(reading);

  return reading;
}