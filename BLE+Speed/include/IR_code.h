#include <Arduino.h>
#ifndef DEFINITIONS_H 
#include <ROAR_Definitions.h>
#endif



int read_IR(int IR_input) {
  int reading = analogRead(IR_input);

  // Serial.println(reading);

  return reading;
}