#include "util.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <PacketSerial.h>
#include <Wire.h>

// clips angle between range of (-180, 180]
double clipAngleto180degrees(double angle) {
    if (angle < 0) angle = 360 + angle;
    angle = fmod(angle, 360);
    return (angle > 180) ? angle - 360 : angle;
}

double clipAngleto360degrees(double angle) {
    angle = fmod(angle, 360);
    return angle < 0 ? angle + 360 : angle;
}

// Trigonometric functions that work in degrees clipped from (-180, 180]

double atand(double y, double x) {
    return clipAngleto180degrees(atan2(y, x) * RAD_TO_DEG);
}

void printDouble(Stream &serial, double value, uint8_t integerPlaces,
                 uint8_t decimalPlaces) {
    const auto integerComponent = (int)value;
    const auto decimalComponent =
        (int)round(abs(value * pow(10, decimalPlaces))) %
        (int)pow(10, decimalPlaces);

    if (integerPlaces == 0)
        serial.printf("%d", integerComponent);
    else
        serial.printf("%*d", integerPlaces, integerComponent);
    serial.print(".");
    if (decimalPlaces == 0)
        serial.printf("%d", decimalComponent);
    else
        serial.printf("%0*d", decimalPlaces, decimalComponent);
}

double sind(double angle) { return sin(angle * DEG_TO_RAD); }
double cosd(double angle) { return cos(angle * DEG_TO_RAD); }

// Wipes the EEPROM.
void wipeEEPROM() {
    Serial.println("Wiping EEPROM...");
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
        Serial.printf("%4d / %d\n", i, EEPROM.length());
    }
    Serial.println("Done");
}