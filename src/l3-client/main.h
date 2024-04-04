#ifndef MAIN_H
#define MAIN_H

#include "BLEDevice.h"
// #include "BLEScan.h"
#include <Arduino.h>

// Define UUIDs:
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID TxAttackModeUUID("1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e");
static BLEUUID RxAttackModeUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID TxPermissiontoSwitchUUID("0001d822-ae4c-496e-bee1-180b573165c4");
static BLEUUID RxPermissiontoSwitchUUID("1c557763-80bc-4fac-8c5c-b0a1ed3298eb");
static BLEUUID TxBallDistanceUUID("cd5c8b4d-920c-41f9-af5a-8cdae7b52c97");
static BLEUUID RxBallDistanceUUID("002c83ff-2f08-46b8-9f27-6ad8b9322d4e");
static BLEUUID TxBallAngleUUID("5076512b-4915-47ba-93e8-318d603f8e39");
static BLEUUID RxBallAngleUUID("d7d8cf6d-d766-4436-ab2c-1005aa04c16e");
// Some variables to keep track on device connected
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

// Define pointer for the BLE connection
static BLEAdvertisedDevice *myDevice;
BLERemoteCharacteristic *TxAttackMode;
BLERemoteCharacteristic *RxAttackMode;
BLERemoteCharacteristic *TxPermissiontoSwitch;
BLERemoteCharacteristic *RxPermissiontoSwitch;
BLERemoteCharacteristic *TxBallDistance;
BLERemoteCharacteristic *RxBallDistance;
BLERemoteCharacteristic *TxBallAngle;
BLERemoteCharacteristic *RxBallAngle;

int attackMode = 0;
// Callback function for Notify function
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData, size_t length, bool isNotify) {
    if (pBLERemoteCharacteristic->getUUID().toString() ==
        RxAttackModeUUID.toString()) {

        // convert received bytes to integer
        // uint32_t counter = pData[0];
        // for(int i = 1; i<length; i++) {
        //   counter = counter | (pData[i] << i*8);
        // }

        // print to Serial
        // Serial.print("Characteristic 1 (Notify) from server: ");
        // Serial.println(counter );
    }
}
// Function to chech Characteristic
bool connectCharacteristic(BLERemoteService *pRemoteService,
                           BLERemoteCharacteristic *l_BLERemoteChar) {
    // Obtain a reference to the characteristic in the service of the remote BLE
    // server.
    if (l_BLERemoteChar == nullptr) {
        Serial.print("Failed to find one of the characteristics");
        Serial.print(l_BLERemoteChar->getUUID().toString().c_str());
        return false;
    }
    Serial.println(" - Found characteristic: " +
                   String(l_BLERemoteChar->getUUID().toString().c_str()));

    if (l_BLERemoteChar->canNotify())
        l_BLERemoteChar->registerForNotify(notifyCallback);

    return true;
}

// Callback function that is called whenever a client is connected or
// disconnected
class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient *pclient) {}

    void onDisconnect(BLEClient *pclient) {
        connected = false;
        Serial.println("onDisconnect");
    }
};

// Function that is run whenever the server is connected
bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    BLEClient *pClient = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice); // if you pass BLEAdvertisedDevice instead of
                                // address, it will be recognized type of peer
                                // device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our service");

    connected = true;
    RxAttackMode = pRemoteService->getCharacteristic(RxAttackModeUUID);
    TxAttackMode = pRemoteService->getCharacteristic(TxAttackModeUUID);
    if (connectCharacteristic(pRemoteService, RxAttackMode) == false)
        connected = false;
    else if (connectCharacteristic(pRemoteService, TxAttackMode) == false)
        connected = false;

    if (connected == false) {
        pClient->disconnect();
        Serial.println("At least one characteristic UUID not found");
        return false;
    }
    return true;
}

// Scan for BLE servers and find the first one that advertises the service we
// are looking for.
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    // Called for each advertising BLE server.
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("BLE Advertised Device found: ");
        Serial.println(advertisedDevice.toString().c_str());

\
        // We have found a device, let us now see if it contains the service we
        // are looking for.
        if (advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(serviceUUID)) {

            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            doScan = true;

        } // Found our server
    }     // onResult
};        // MyAdvertisedDeviceCallbacks

#endif