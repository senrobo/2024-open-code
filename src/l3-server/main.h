#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>


// Initialize all pointers
BLEServer *pServer = NULL;              
BLECharacteristic *TxAttackMode = NULL;
BLECharacteristic *RxAttackMode = NULL;
BLECharacteristic *TxPermissiontoSwitch = NULL;
BLECharacteristic *RxPermissiontoSwitch = NULL;
BLECharacteristic *TxBallDistance = NULL;
BLECharacteristic *RxBallDistance = NULL;
BLECharacteristic *TxBallAngle = NULL;
BLECharacteristic *RxBallAngle = NULL;
BLEDescriptor *pDescr_1; 
BLE2902 *pBLE2902_1;    
BLE2902 *pBLE2902_2;    

// Some variables to keep track on device connected
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Variable that will continuously be increased and written to the client
uint32_t value = 0;
int attackMode = 0;

static BLEUUID SERVICE_UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID TxAttackModeUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID RxAttackModeUUID("1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e");
static BLEUUID TxPermissiontoSwitchUUID("1c557763-80bc-4fac-8c5c-b0a1ed3298eb");
static BLEUUID RxPermissiontoSwitchUUID("0001d822-ae4c-496e-bee1-180b573165c4");
static BLEUUID TxBallDistanceUUID("002c83ff-2f08-46b8-9f27-6ad8b9322d4e");
static BLEUUID RxBallDistanceUUID("cd5c8b4d-920c-41f9-af5a-8cdae7b52c97");
static BLEUUID TxBallAngleUUID("d7d8cf6d-d766-4436-ab2c-1005aa04c16e");
static BLEUUID RxBallAngleUUID("5076512b-4915-47ba-93e8-318d603f8e39");

class MyServerCallbacks : public BLEServerCallbacks {

    void onConnect(BLEServer *pServer) { deviceConnected = true; };

    void onDisconnect(BLEServer *pServer) { deviceConnected = false; }
};

#endif