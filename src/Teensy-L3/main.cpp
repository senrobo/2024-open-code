#include "PacketSerial.h"
#include "SPI.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "ballposition.h"
#include "config.h"
#include "kalman.h"
#include "sensorfusion.h"
#include "vector.h"
#include <Arduino.h>
#include <PacketSerial.h>
#include <Wire.h>
#include <iostream>
#include <math.h>

#define TEENSY

#define LDRPINCOUNT 36
#define RadiusofLDR 1.0F

Sensorfusion sensorfusion;
BallPosition ballposition;

PacketSerial CameraTeensySerial;
PacketSerial TeensyTeensySerial;
PacketSerial LidarTeensySerial;
PacketSerial BluetoothTeensySerial;
BNO08x bno;

SensorValues sensorValues;
ProcessedValues processedValues;
BluetoothData bluetoothData;
TeensyBluetooth teensyBluetooth;
void receiveLidarTxData(const byte *buf, size_t size) {
    // load payload
    LidarTxPayload payload;
    // if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));
    for (int i = 0; i < 4; i++) {
        sensorValues.lidardist[i] = payload.lidarTxData.distance[i];
    }
    return;
}

void receiveBluetoothTxData(const byte *buf, size_t size) {
    // load payload
    BluetoothTxPayload payload;
    // if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));
    processedValues.attackMode = payload.bluetoothData.attackMode;
    return;
}

void receiveCameraTxData(const byte *buf, size_t size) {
    // load payload
    CameraTxPayload payload;
    // if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));

#ifdef YELLOW_GOAL_ATTACK
    sensorValues.bluegoal_relativeposition.angle =
        payload.cameraTxData.values[0];
    sensorValues.bluegoal_relativeposition.distance =
        payload.cameraTxData.values[1];
    sensorValues.yellowgoal_relativeposition.angle =
        payload.cameraTxData.values[2];
    sensorValues.yellowgoal_relativeposition.distance =
        payload.cameraTxData.values[3];
#endif
#ifdef BLUE_GOAL_ATTACK
    sensorValues.yellowgoal_relativeposition.angle =
        payload.cameraTxData.values[0];
    sensorValues.yellowgoal_relativeposition.distance =
        payload.cameraTxData.values[1];
    sensorValues.bluegoal_relativeposition.angle =
        payload.cameraTxData.values[2];
    sensorValues.bluegoal_relativeposition.distance =
        payload.cameraTxData.values[3];
#endif
    sensorValues.ball_relativeposition.angle = payload.cameraTxData.values[4];
    sensorValues.ball_relativeposition.distance =
        payload.cameraTxData.values[5];
    sensorValues.ball_relativeposition.distance =
        ballMirrorMapping(sensorValues.ball_relativeposition.distance);
    sensorValues.bluegoal_relativeposition.distance =
        frontMirrorMapping(sensorValues.bluegoal_relativeposition.distance);
    sensorValues.yellowgoal_relativeposition.distance =
        frontMirrorMapping(sensorValues.yellowgoal_relativeposition.distance);
    return;
}

void getBNOreading() {
    // bno.enableGyroIntegratedRotationVector();
    bno.enableGameRotationVector();
    if (bno.getSensorEvent() == true) {
        if (bno.getSensorEventID() ==
            SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
                if (bno.getGameQuatK() <= 0){
                    sensorValues.relativeBearing = -(- bno.getGameQuatK() * 180 + 180 - bno.getGameQuatReal() * 180) / 2;
                }
                else{
                    sensorValues.relativeBearing =
                        -(-bno.getGameQuatK() * 180 - 180 + bno.getGameQuatReal() * 180) / 2;
                }
        }
    }
}

void setReports(void) {

    if (bno.enableGameRotationVector() == true) {
        // Serial.println(F("Gryo Integrated Rotation vector enabled"));
        // Serial.println(F("Output in form i, j, k, real, gyroX, gyroY,
        // gyroZ"));
    } else {
        // Serial.println("Could not enable gyro integrated rotation vector");
    }
}

Vector localize() {
    if ((sensorValues.yellowgoal_relativeposition.distance < 90 &&
         processedValues.yellowgoal_exists == 1) ||
        (processedValues.yellowgoal_exists == 1 &&
         processedValues.bluegoal_exists == 0)) {
        sensorfusion.updateSensorValues(
            0, 0, 0, 0, sensorValues.lidardist[0], sensorValues.lidardist[2],
            sensorValues.lidardist[3], sensorValues.lidardist[1],
            localizeWithOffensiveGoal().x(), localizeWithOffensiveGoal().y());
        Vector localisation = sensorfusion.updateLocalisation();
        Serial.print("f");
        return localisation;
    } else if ((sensorValues.bluegoal_relativeposition.distance < 90 &&
                processedValues.bluegoal_exists == 1) ||
               (processedValues.bluegoal_exists == 1 &&
                processedValues.yellowgoal_exists == 0)) {
        sensorfusion.updateSensorValues(
            0, 0, 0, 0, sensorValues.lidardist[0], sensorValues.lidardist[2],
            sensorValues.lidardist[3], sensorValues.lidardist[1],
            localizeWithDefensiveGoal().x(), localizeWithDefensiveGoal().y());
        Vector localisation = sensorfusion.updateLocalisation();
        Serial.print("b");
        return localisation;
    } else {
        sensorfusion.updateSensorValues(
            0, 0, 0, 0, sensorValues.lidardist[0], sensorValues.lidardist[2],
            sensorValues.lidardist[3], sensorValues.lidardist[1],
            localizeWithBothGoals().x(), localizeWithBothGoals().y());
        Vector localisation = sensorfusion.updateLocalisation();
        Serial.print("fb");
        return localisation;
    }
    // sensorfusion.updateSensorValues(movement.getmotorValues()[0],movement.getmotorValues()[1],movement.getmotorValues()[2],
    //                                   movement.getmotorValues()[3], 0,
    //                                   sensorValues.lidardist[3], 0, 0,
    //                                   localizeWithDefensiveGoal().x(),
    //                                   localizeWithDefensiveGoal().y());
}

void verifyingObjectExistance() {
    processedValues.ballExists =
        (processedValues.ball_relativeposition.distance == 0) ? 0 : 1;
    processedValues.bluegoal_exists =
        (sensorValues.bluegoal_relativeposition.distance == 0) ? 0 : 1;
    processedValues.yellowgoal_exists =
        (sensorValues.yellowgoal_relativeposition.distance == 0) ? 0 : 1;
}

void setup() {
    Serial1.begin(500000);
    Serial5.begin(500000);
    Serial4.begin(115200);
    Serial3.begin(115200); // bluetooth
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(1000000);
    bno.begin(0x4A, Wire);
    CameraTeensySerial.begin(&Serial5);
    CameraTeensySerial.setPacketHandler(&receiveCameraTxData);

    LidarTeensySerial.begin(&Serial4);
    LidarTeensySerial.setPacketHandler(&receiveLidarTxData);

    TeensyTeensySerial.begin(&Serial1);

    BluetoothTeensySerial.begin(&Serial3);
    BluetoothTeensySerial.setPacketHandler(&receiveBluetoothTxData);

    setReports();

    analogWriteResolution(10);

    // byte buf[sizeof(l1TxPayload)];
    // memcpy(buf,&SAMDlinedata, sizeof(SAMDlinedata));
    // L1Serial.send(buf,sizeof(buf));
}

int counter = 0;
double frontVariance = 400;
double backVariance = 400;
double leftVariance = 400;
double rightVariance = 400;

void loop() {
    double dt = loopTimeinMillis();
    CameraTeensySerial.update();
    LidarTeensySerial.update();
    BluetoothTeensySerial.update();

    setReports();
    getBNOreading();

    teensyBluetooth.currentMode = processedValues.attackMode;
    

    processedValues.bluegoal_relativeposition =
        sensorValues.bluegoal_relativeposition;
    processedValues.bluegoal_relativeposition.distance =
        sensorValues.bluegoal_relativeposition.distance;
    processedValues.yellowgoal_relativeposition =
        sensorValues.yellowgoal_relativeposition;
    processedValues.yellowgoal_relativeposition.distance =
        sensorValues.yellowgoal_relativeposition.distance;
    processedValues.relativeBearing = -sensorValues.relativeBearing;

    // setup
    verifyingObjectExistance();
    processLidars();

    (processedValues.lidarConfidence[0] == 1) ? frontVariance = 3
                                              : frontVariance = 100000;
    (processedValues.lidarConfidence[1] == 1) ? rightVariance = 3
                                              : rightVariance = 100000;
    (processedValues.lidarConfidence[2] == 1) ? backVariance = 3
                                              : backVariance = 100000;
    (processedValues.lidarConfidence[3] == 1) ? leftVariance = 0
                                              : leftVariance = 100000;

    sensorfusion.updateConstants(frontVariance, backVariance, leftVariance,
                                 rightVariance, 10, 15);

    ballposition.updateConstants(dt / 1000);
    ballposition.updateSensorMeasurement(
        sensorValues.ball_relativeposition.x(),
        sensorValues.ball_relativeposition.y());
    processedValues.ball_relativeposition = ballposition.updatePosition();

    Vector robotPosition = localize();
    processedValues.robot_position = {robotPosition.x(), robotPosition.y()};

    if (teensyBluetooth.currentMode == 0 && 
    processedValues.ball_relativeposition.angle < 20 && 
    processedValues.ball_relativeposition.angle > -20 &&
    processedValues.ball_relativeposition.distance < 30 &&
    processedValues.ballExists == 1){
        teensyBluetooth.switchMode = 1;
    }
    else{
        teensyBluetooth.switchMode = 0;
    }
    if (teensyBluetooth.currentMode == 0 && teensyBluetooth.switchMode == 1){
        processedValues.attackMode = 1;
    }

    Serial.print(" | bearing: ");
    printDouble(Serial, processedValues.relativeBearing, 3, 1);
    Serial.print(" | frontLidar: ");
    printDouble(Serial, processedValues.lidarDistance[0], 3, 0);
    Serial.print(" | rightLidar: ");
    printDouble(Serial, processedValues.lidarDistance[1], 3, 0);
    Serial.print(" | backLidar: ");
    printDouble(Serial, processedValues.lidarDistance[2], 3, 0);
    Serial.print(" | leftLidar: ");
    printDouble(Serial, processedValues.lidarDistance[3], 3, 0);

    Serial.print(" | frontLidarConf: ");
    printDouble(Serial, processedValues.lidarConfidence[0], 3, 0);
    Serial.print(" | rightLidarConf: ");
    printDouble(Serial, processedValues.lidarConfidence[1], 3, 0);
    Serial.print(" | backLidarConf: ");
    printDouble(Serial, processedValues.lidarConfidence[2], 3, 0);
    Serial.print(" | leftLidarConf: ");
    printDouble(Serial, processedValues.lidarConfidence[3], 3, 0);

    Serial.print(" | attackGoalAngle: ");
    printDouble(Serial, processedValues.yellowgoal_relativeposition.angle, 3,
                1);
    Serial.print(" | attackGoalDist: ");
    printDouble(Serial, processedValues.yellowgoal_relativeposition.distance, 3,
                1);
    Serial.print(" | defenceGoalAngle: ");
    printDouble(Serial, processedValues.bluegoal_relativeposition.angle, 3, 1);
    Serial.print(" | defenceGoalDist: ");
    printDouble(Serial, processedValues.bluegoal_relativeposition.distance, 3,
                1);
    Serial.print(" | ballAngle: ");
    printDouble(Serial, processedValues.ball_relativeposition.angle, 3, 1);
    Serial.print(" | ballDist: ");
    printDouble(Serial, processedValues.ball_relativeposition.distance, 3, 1);
    // processed Values
    Serial.print(" | ballExistence: ");
    printDouble(Serial, processedValues.ballExists, 1, 1);
    Serial.print(" | attackGoal Existence: ");
    printDouble(Serial, processedValues.yellowgoal_exists, 1, 1);
    Serial.print(" | defenceGoal Existence: ");
    printDouble(Serial, processedValues.bluegoal_exists, 1, 1);

    Serial.print(" | currentMode: ");
    printDouble(Serial, teensyBluetooth.currentMode, 1, 1);
    Serial.print(" | attackModeESP32: ");
    printDouble(Serial, processedValues.attackMode, 1, 1);
    Serial.print(" | switchMode: ");
    printDouble(Serial, teensyBluetooth.switchMode, 1, 1);

    // Location
    Serial.print(" | X_position: ");
    printDouble(Serial, processedValues.robot_position.x, 3, 0);
    Serial.print(" | Y_position: ");
    printDouble(Serial, processedValues.robot_position.y, 3, 0);
    Serial.println("");

    byte buf[sizeof(teensytoTeensyTxPayload)];
    memcpy(buf, &processedValues, sizeof(processedValues));
    TeensyTeensySerial.send(buf, sizeof(buf));

    byte buf1[sizeof(teensytoBluetoothPayload)];
    memcpy(buf1, &teensyBluetooth, sizeof(teensyBluetooth));
    BluetoothTeensySerial.send(buf1, sizeof(buf1));

    counter++;
}