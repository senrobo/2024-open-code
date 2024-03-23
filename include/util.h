#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>
#include <Wire.h>

#define RADIANS_TO_DEGREES

double clipAngleto180degrees(double angle);
double clipAngleto360degrees(double angle);
double atand(double y, double x);
double sind(double angle);
double cosd(double angle);
double acosd(double angle);

void printDouble(Stream &serial, double value, uint8_t integerPlaces,
                 uint8_t decimalPlaces);

void i2cscanner(Stream &serial, TwoWire wire);

void wipeEEPROM();

#endif