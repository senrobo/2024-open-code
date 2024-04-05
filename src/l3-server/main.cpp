#include <Arduino.h>
#include <HardwareSerial.h>
#include <TFLI2C.h>
#include <Wire.h>

#include "PacketSerial.h"
#include "SPI.h"
#include "main.h"
#include "util.h"

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

struct teensyTxPayload{
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


        BLEDevice::init("ESP32");


    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

 
    BLEService *pService = pServer->createService(SERVICE_UUID);


    TxAttackMode = pService->createCharacteristic(
        TxAttackModeUUID, BLECharacteristic::PROPERTY_NOTIFY);

    RxAttackMode = pService->createCharacteristic(
        RxAttackModeUUID, BLECharacteristic::PROPERTY_READ |
                              BLECharacteristic::PROPERTY_WRITE |
                              BLECharacteristic::PROPERTY_NOTIFY);


    pDescr_1 = new BLEDescriptor((uint16_t)0x2901);
    pDescr_1->setValue("A very interesting variable");
    TxAttackMode->addDescriptor(pDescr_1);

    pBLE2902_1 = new BLE2902();
    pBLE2902_1->setNotifications(true);
    TxAttackMode->addDescriptor(pBLE2902_1);

    pBLE2902_2 = new BLE2902();
    pBLE2902_2->setNotifications(true);
    RxAttackMode->addDescriptor(pBLE2902_2);

 
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(
        0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    Serial.println("Waiting a client connection to notify...");
}

void loop() {
    L3BluetoothSerial.update();
    if (deviceConnected) {

        u_int8_t *rxValue = RxAttackMode->getData();

        if (*rxValue == 1) {
            if (teensyData.switchMode == 1 && teensyData.currentMode == 0) {
                bluetoothData.attackMode = 1;
            } else {
                bluetoothData.attackMode = 0;
            }
        } else {

            bluetoothData.attackMode = 1;
        }
        Serial.print("Characteristic 2 (getValue): ");
        Serial.println(*rxValue);


    }

    if (deviceConnected == false) {bluetoothData.attackMode = 1; }

    // Disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things
        pServer->startAdvertising(); 
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // Connecting
    if (deviceConnected && !oldDeviceConnected) {
       
        oldDeviceConnected = deviceConnected;
    }


    Serial.print(bluetoothData.attackMode);
    Serial.print(", ");
    Serial.print(teensyData.currentMode);
    Serial.print(", ");
    Serial.println(teensyData.switchMode);
    byte buf[sizeof(bluetoothTxPayload)];
    memcpy(buf, &bluetoothData, sizeof(bluetoothData));
    L3BluetoothSerial.send(buf, sizeof(buf));
}
