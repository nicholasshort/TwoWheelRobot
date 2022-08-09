#include <Arduino.h>

#define PIN_D2 5 

void setup() {
  pinMode(PIN_D2, OUTPUT);
}

void loop() {
  digitalWrite(PIN_D2, HIGH);
}