#include <Arduino.h>
#ifndef DEFINITIONS_H 
#include <ROAR_Definitions.h>
#endif



int check_IR() {
  int reading = analogRead(IR);

  Serial.println(reading);

  return reading;
}