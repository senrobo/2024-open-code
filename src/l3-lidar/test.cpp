// #include <Arduino.h>
// #include <HardwareSerial.h>
// #include <TFLI2C.h>
// #include <Wire.h>

// #include "PacketSerial.h"
// #include "SPI.h"
// #include "main.h"
// #include "util.h"

// HardwareSerial MySerial0(0);

// #define XSHUT1PIN 0

// PacketSerial L3LIDARSerial;
// TFLI2C front;
// TFLI2C right;
// TFLI2C back;
// TFLI2C left;
// TFLI2C tflI2C[4] = {front, right, back, left};

// int16_t tfDist[4];

// int tfAddress[4] = {0x35, 0x22, 0x33, 0x44}; // front, right, back, left
// // int tfAddr[4] = {0x11, 0x22, 0x33, 0x44}; // front, right, back, left
// struct lidardata {
//     int distance[4];
//     int attackMode = 0;
// };
// lidardata esp32lidardata;

// typedef struct lidarTxPayload {
//     lidardata esp32lidardata;
// } lidarTxPayload;

// struct teensydata {
//     int switchModes = 0;
// };
// teensydata teensyData;

// void setup() {
//     Serial.begin(115200);
//     MySerial0.begin(115200, SERIAL_8N1, -1, -1);

//     L3LIDARSerial.begin(&MySerial0);
//     Wire.begin();
//     // tflI2C[2].Set_I2C_Addr(0x22,0x10);
//     // // tflI2C[3].Soft_Reset(0x44);
//     // tflI2C[2].Soft_Reset(0x22);
//     for (int i = 0; i < 4; i++) { tflI2C[i].Save_Settings(tfAddress[i]); }

//     BLEDevice::init("ESP32");

//     // Create the BLE Server
//     pServer = BLEDevice::createServer();
//     pServer->setCallbacks(new MyServerCallbacks());

//     // Create the BLE Service
//     BLEService *pService = pServer->createService(SERVICE_UUID);

//     // Create a BLE Characteristic
//     TxAttackMode = pService->createCharacteristic(
//         TxAttackModeUUID, BLECharacteristic::PROPERTY_NOTIFY);

//     RxAttackMode = pService->createCharacteristic(
//         RxAttackModeUUID, BLECharacteristic::PROPERTY_READ |
//                               BLECharacteristic::PROPERTY_WRITE |
//                               BLECharacteristic::PROPERTY_NOTIFY);

//     // Create a BLE Descriptor
//     pDescr_1 = new BLEDescriptor((uint16_t)0x2901);
//     pDescr_1->setValue("A very interesting variable");
//     TxAttackMode->addDescriptor(pDescr_1);

//     // Add the BLE2902 Descriptor because we are using "PROPERTY_NOTIFY"
//     pBLE2902_1 = new BLE2902();
//     pBLE2902_1->setNotifications(true);
//     TxAttackMode->addDescriptor(pBLE2902_1);

//     pBLE2902_2 = new BLE2902();
//     pBLE2902_2->setNotifications(true);
//     RxAttackMode->addDescriptor(pBLE2902_2);

//     // Start the service
//     pService->start();

//     // Start advertising
//     BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//     pAdvertising->addServiceUUID(SERVICE_UUID);
//     pAdvertising->setScanResponse(false);
//     pAdvertising->setMinPreferred(
//         0x0); // set value to 0x00 to not advertise this parameter
//     BLEDevice::startAdvertising();
//     Serial.println("Waiting a client connection to notify...");
// }

// void loop() {

//     for (int i = 0; i < 4; i++) {
//         if (tflI2C[i].getData(tfDist[i], tfAddress[i])) // If read okay...
//         {
//             if (i < 3) {
//                 Serial.print(" Dist: ");
//                 Serial.print(tfDist[i]);
//             }

//             else if (i == 3) {
//                 Serial.print(" Dist: ");
//                 Serial.println(tfDist[i]);
//             }
//             esp32lidardata.distance[i] = tfDist[i];
//         } else {
//             esp32lidardata.distance[i] = 0;
//             tflI2C[i].printStatus();
//         }
//     }
//     if (deviceConnected) {

//         u_int8_t *rxValue = RxAttackMode->getData();

//         if (*rxValue == 1) {
//             if (teensyData.switchModes == 1) {
//                 esp32lidardata.attackMode = 1;
//             } else {
//                 esp32lidardata.attackMode = 0;
//             }
//         } else {

//             esp32lidardata.attackMode = 1;
//         }
//         Serial.print("Characteristic 2 (getValue): ");
//         Serial.println(*rxValue);

//         delay(100);
//     }

//     if (deviceConnected == false) { esp32lidardata.attackMode = 1; }

//     // Disconnecting
//     if (!deviceConnected && oldDeviceConnected) {
//         delay(500); // give the bluetooth stack the chance to get things ready
//         pServer->startAdvertising(); // restart advertising
//         Serial.println("start advertising");
//         oldDeviceConnected = deviceConnected;
//     }
//     // Connecting
//     if (deviceConnected && !oldDeviceConnected) {
//         // do stuff here on connecting
//         oldDeviceConnected = deviceConnected;
//     }

//     byte buf[sizeof(lidarTxPayload)];
//     memcpy(buf, &esp32lidardata, sizeof(esp32lidardata));
//     L3LIDARSerial.send(buf, sizeof(buf));
// }
