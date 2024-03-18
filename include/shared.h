#ifndef SHARED_H
#define SHARED_H

#include "vector.h"

// shared values
struct CameraTxData {
    double values[6];
};

struct CameraTxPayload {
    CameraTxData cameraTxData;
};

struct L1TxData{
  int onLine = 0;
  float angleBisector;
  float depthinLine;
  int linetrackldr1;
  int linetrackldr2;
  int ballinCatchment;
//   int SerialLDRID = 0;
//   int SerialLDRvalue = 0;
};

struct L1TxPayload {
    L1TxData l1TxData;
};

struct SensorValues {
    // camera values
    Vector yellowgoal_relativeposition;
    Vector bluegoal_relativeposition;
    Vector ball_relativeposition;
    double relativeBearing;
    // l1 values
    int onLine = 0;
    float angleBisector;
    float depthinLine;
    int linetrackldr1;
    int linetrackldr2;
    int ballinCatchment;
    int lidardist[4];
    // int SerialLDRID = 0;
    // int SerialLDRvalue = 0;
};

struct L3sensorValues {
    int relativeBearing;
    Vector yellowgoal_relativeposition;
    Vector bluegoal_relativeposition;
    Vector ball_relativeposition;
    int lidardist[4];
};

struct L3Txpayloaf {
    L3sensorValues sensorvalues;
};

struct Solenoid {
    int kick = 5;
};

struct L2TxtoL1payload {
    Solenoid solenoid;
};

struct TimeControl {
    double now = 0;
    double last = 0;
    double dt = 0;
};

struct ProcessedValues {
    Vector ball_relativeposition;
    int ballExists = 0;
    int yellowgoal_exists = 0;
    int bluegoal_exists = 0;
    int lidarDistance[4];
    double lidarConfidence[4];
    int ballinCatchment = 0;
    double defenceRobotAngleBisector;
    double defenceRobotDepthinLine;
};

struct TruthValues {
    double bearingtoField;
    Vector ballPositiontoField;
};

struct Execution {
    double targetBearing = 0;
    int strategy = 1;
    int setStrategy = 1;
    int kickStrategySequence = 0;
    int kickComplete = 0;
};

static TimeControl timeControl;

#define DRIBBLERPWM 0

#endif