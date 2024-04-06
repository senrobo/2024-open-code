#include <Arduino.h>
#include <HardwareSerial.h>
#include <TFLI2C.h>
#include <Wire.h>

#include "PacketSerial.h"
#include "SPI.h"
#include "util.h"
#include "main.h"

HardwareSerial MySerial0(0);

#define XSHUT1PIN 0

PacketSerial L3BluetoothSerial;

struct BluetoothData {
    int attackMode = 0;
};
BluetoothData bluetoothData;

typedef struct BluetoothTxPayload {
    BluetoothData bluetoothData;
} bluetoothTxPayload;

struct teensydata {
    int switchMode = 0;
    int currentMode = 0;
};

struct teensyTxPayload {
    teensydata teensyData;
};
teensydata teensyData;

void receiveTeensyTxData(const byte *buf, size_t size) {
    // load payload
    teensyTxPayload payload;
    // if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));
    teensyData.currentMode = payload.teensyData.currentMode;
    teensyData.switchMode = payload.teensyData.switchMode;
    return;
}

void setup() {
    Serial.begin(115200);
    MySerial0.begin(115200, SERIAL_8N1, -1, -1);

    L3BluetoothSerial.begin(&MySerial0);
    L3BluetoothSerial.setPacketHandler(&receiveTeensyTxData);

    BLEDevice::init("");

    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
}

void loop() {
    L3BluetoothSerial.update();

    (connected) ? doConnect == false : doConnect == true;

    if (doConnect == true) {
        if (connectToServer()) {
            Serial.println("We are now connected to the BLE Server.");
        } else {
            Serial.println("We have failed to connect to the server; there is "
                           "nothin more we will do.");
        }
        doConnect = false;
    }


    if (connected) {

        u_int16_t rxValue = RxAttackMode->readUInt16();
        Serial.print("Characteristic 2 (readValue): ");
        Serial.println(rxValue);
        // if (rxValue == 1) {
        //     if (teensyData.switchMode == 1 && teensyData.currentMode == 0) {
        //         bluetoothData.attackMode = 1;
        //     } else {
        //         bluetoothData.attackMode = 0;
        //     }
        // } else {

        //     bluetoothData.attackMode = 1;
        // }
        bluetoothData.attackMode = 1;

        u_int8_t txValue = bluetoothData.attackMode;

        TxAttackMode->writeValue(txValue, false);
    } else {
        bluetoothData.attackMode = 1;
    }
    delay(5);

    byte buf[sizeof(bluetoothTxPayload)];
    memcpy(buf, &bluetoothData, sizeof(bluetoothData));
    L3BluetoothSerial.send(buf, sizeof(buf));

    if (doScan && connected == false) {
        byte buf[sizeof(bluetoothTxPayload)];
        memcpy(buf, &bluetoothData, sizeof(bluetoothData));
        L3BluetoothSerial.send(buf, sizeof(buf));
        BLEDevice::getScan()->start(
            0); // this is just example to start scan after disconnect, most
                // likely there is better way to do it in arduino
    }
    // Serial.print(bluetoothData.attackMode);
    // Serial.print(", ");
    // Serial.print(teensyData.currentMode);
    // Serial.print(", ");
    // Serial.println(teensyData.switchMode);

}
