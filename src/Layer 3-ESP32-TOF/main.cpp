#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
// put function declarations here:
int myFunction(int, int);

void setup() {
    Serial.begin(9600);
}

void loop() {
  Serial.print("Side Teensy");
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}