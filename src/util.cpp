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

void i2cscanner() {
    byte error, address;
    int nDevices = 0;
    // foresight of future problems ig
    Serial.println("Scanning...");

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("i2c device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX); // ASCII-encoded hexadecimal
            Serial.println();
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    nDevices == 0 ? Serial.println("No i2c devices found\n")
                  : Serial.println("Scanning completed\n");
    delay(1000);
}

// Wipes the EEPROM.
void wipeEEPROM() {
    Serial.println("Wiping EEPROM...");
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
        Serial.printf("%4d / %d\n", i, EEPROM.length());
    }
    Serial.println("Done");
}