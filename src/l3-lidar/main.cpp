#include <Arduino.h>
#include <HardwareSerial.h>
#include <TFLI2C.h>
#include <Wire.h>

#include "PacketSerial.h"
#include "SPI.h"

HardwareSerial MySerial0(0);

#define XSHUT1PIN 0

PacketSerial L3LIDARSerial;
TFLI2C front;
TFLI2C right;
TFLI2C back;
TFLI2C left;
TFLI2C tflI2C[4] = {front, right, back, left};

int16_t tfDist[4];

int tfAddress[4] = {0x35, 0x22, 0x33, 0x44}; // front, right, back, left
// int tfAddr[4] = {0x11, 0x22, 0x33, 0x44}; // front, right, back, left
struct lidardata {
    int distance[4];
};
lidardata esp32lidardata;

typedef struct lidarTxPayload {
    lidardata esp32lidardata;
} lidarTxPayload;

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

void setup() {
    Serial.begin(115200);
    MySerial0.begin(115200, SERIAL_8N1, -1, -1);

    L3LIDARSerial.begin(&MySerial0);
    Wire.begin();
    // tflI2C.Set_I2C_Addr(0x11,0x35);
    // tflI2C.Soft_Reset(0x11);
    for (int i = 0; i < 4; i++) { tflI2C[i].Save_Settings(tfAddress[i]); }
}

void loop() {
    // i2cscanner();
    for (int i = 0; i < 4; i++) {
        if (tflI2C[i].getData(tfDist[i], tfAddress[i])) // If read okay...
        {
            if (i < 3) {
                Serial.print(" Dist: ");
                Serial.print(tfDist[i]);
            }

            else if (i == 3) {
                Serial.print(" Dist: ");
                Serial.println(tfDist[i]);
            }
            esp32lidardata.distance[i] = tfDist[i];
        }

        else {
            esp32lidardata.distance[i] = 0;
            tflI2C[i].printStatus();
        }
    }
    delay(20);

    byte buf[sizeof(lidarTxPayload)];
    memcpy(buf, &esp32lidardata, sizeof(esp32lidardata));
    L3LIDARSerial.send(buf, sizeof(buf));
}
