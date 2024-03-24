#ifndef MAIN_H
#define MAIN_H
#include <Arduino.h>
#include <ArduinoEigenDense.h>

#include <array>

#include "PacketSerial.h"
#include "config.h"
#include "util.h"
#include "vector.h"

struct ProcessedValues {
    Vector ball_relativeposition;
    Vector yellowgoal_relativeposition;
    Vector bluegoal_relativeposition;
    int ballExists = 0;
    int yellowgoal_exists = 0;
    int bluegoal_exists = 0;
    int lidarDistance[4];
    double lidarConfidence[4];
    int relativeBearing;
    Point robot_position;
};

struct SensorValues {
    int relativeBearing;
    Vector yellowgoal_relativeposition;
    Vector bluegoal_relativeposition;
    Vector ball_relativeposition;
    int lidardist[4];
};


extern SensorValues sensorValues;
extern ProcessedValues processedValues;


struct teensytoTeensyTxPayload {
    ProcessedValues processedValues;
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





//subroutines
void verifyingObjectExistance();
void processLidars();
Vector localizeWithOffensiveGoal();
Vector localizeWithDefensiveGoal();
Vector localizeWithBothGoals();
double frontMirrorMapping(double distance);
double ballMirrorMapping(double distance);
#endif