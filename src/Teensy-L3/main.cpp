#include "PacketSerial.h"
#include "SPI.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "vector.h"
#include <Arduino.h>
#include <PacketSerial.h>
#include <Wire.h>
#include <iostream>
#include <math.h>

#define TEENSY

#define LDRPINCOUNT 36
#define RadiusofLDR 1.0F

PacketSerial CameraTeensySerial;
PacketSerial TeensyTeensySerial;
PacketSerial LidarTeensySerial;
BNO08x bno;

struct sensorValues {
    int relativeBearing;
    Vector yellowgoal_relativeposition;
    Vector bluegoal_relativeposition;
    Vector ball_relativeposition;
    int LidarDist[4];
};

struct teensytoTeensyTxPayload {
    sensorValues sensorvalues;
};

struct CameraTxData {
    double values[6];
};

struct CameraTxPayload {
    CameraTxData cameraTxData;
};

struct LidarTxData {
    int distance[4];
};

struct LidarTxPayload {
    LidarTxData lidarTxData;
};

sensorValues sensorvalues;

void receiveLidarTxData(const byte *buf, size_t size) {
    // load payload
    LidarTxPayload payload;
    // if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));
    for (int i = 0; i < 4; i++) {
        sensorvalues.LidarDist[i] = payload.lidarTxData.distance[i];
    }
    return;
}

void receiveCameraTxData(const byte *buf, size_t size) {
    // load payload
    CameraTxPayload payload;
    // if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));
    sensorvalues.bluegoal_relativeposition.angle =
        payload.cameraTxData.values[0];
    sensorvalues.bluegoal_relativeposition.distance =
        payload.cameraTxData.values[1];
    sensorvalues.yellowgoal_relativeposition.angle =
        payload.cameraTxData.values[2];
    sensorvalues.yellowgoal_relativeposition.distance =
        payload.cameraTxData.values[3];
    sensorvalues.ball_relativeposition.angle = payload.cameraTxData.values[4];
    sensorvalues.ball_relativeposition.distance =
        payload.cameraTxData.values[5];
    return;
}

void getBNOreading() {
    // bno.enableGyroIntegratedRotationVector();
    bno.enableGyroIntegratedRotationVector();
    if (bno.getSensorEvent() == true) {
        if (bno.getSensorEventID() ==
            SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR) {
            sensorvalues.relativeBearing =
                bno.getGyroIntegratedRVK() *
                180.0; // Convert yaw / heading to degree
        }
    }
}

void setReports(void) {
    Serial.println("Setting desired reports");
    if (bno.enableGyroIntegratedRotationVector() == true) {
        // Serial.println(F("Gryo Integrated Rotation vector enabled"));
        // Serial.println(F("Output in form i, j, k, real, gyroX, gyroY,
        // gyroZ"));
    } else {
        // Serial.println("Could not enable gyro integrated rotation vector");
    }
}

void setup() {
    Serial1.begin(115200);
    Serial5.begin(115200);
    Serial4.begin(115200);
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(100000);
    bno.begin(0x4A, Wire);
    CameraTeensySerial.begin(&Serial5);
    CameraTeensySerial.setPacketHandler(&receiveCameraTxData);

    LidarTeensySerial.begin(&Serial4);
    LidarTeensySerial.setPacketHandler(&receiveLidarTxData);

    TeensyTeensySerial.begin(&Serial1);

    setReports();

    analogWriteResolution(10);

    // byte buf[sizeof(l1TxPayload)];
    // memcpy(buf,&SAMDlinedata, sizeof(SAMDlinedata));
    // L1Serial.send(buf,sizeof(buf));
}

int counter = 0;

void loop() {

    setReports();
    getBNOreading();
    CameraTeensySerial.update();
    LidarTeensySerial.update();
    Serial.print(sensorvalues.relativeBearing);
    Serial.print(", ");
    Serial.print(sensorvalues.bluegoal_relativeposition.distance);
    Serial.print(", ");
    Serial.print(sensorvalues.LidarDist[0]);
    Serial.print(", ");
    Serial.print(sensorvalues.LidarDist[1]);
    Serial.print(", ");
    Serial.print(sensorvalues.LidarDist[2]);
    Serial.print(", ");
    Serial.print(sensorvalues.LidarDist[3]);
    Serial.print(", ");
    Serial.println(" ");

    byte buf[sizeof(teensytoTeensyTxPayload)];
    memcpy(buf, &sensorvalues, sizeof(sensorvalues));
    TeensyTeensySerial.send(buf, sizeof(buf));

    // printHighestValue(1);

    // autoTuneThreshold(5000, 0.7);

    counter++;
}