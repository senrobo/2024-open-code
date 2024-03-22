#ifndef SHARED_H
#define SHARED_H

#include "vector.h"

// shared values

#define LDRPINCOUNT 36

struct LightArray{
int RAWLDRVALUES[LDRPINCOUNT] = {
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00};
int LDRPINMap[LDRPINCOUNT]{0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
                           12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24,
                           25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};

float LDRBearings[LDRPINCOUNT]{
    0.00,   10.00,  20.00,  30.00,  40.00,  50.00,  60.00,  70.00,
    80.00,  90.00,  100.00, 110.00, 120.00, 130.00, 140.00, 150.00,
    160.00, 170.00, 180.00, 190.00, 200.00, 210.00, 220.00, 230.00,
    240.00, 250.00, 260.00, 270.00, 280.00, 290.00, 300.00, 310.00,
    320.00, 330.00, 340.00, 350.00}; // assuming placement of ldrs is constant
float LDRThresholds[LDRPINCOUNT]{
    870.00, 873.50, 849.00, 862.00, 860.00, 850.00, 848.00, 857.50, 859.00,
    849.00, 847.00, 856.50, 861.50, 860.50, 868.00, 865.00, 867.00, 879.00,
    872.50, 863.00, 861.50, 869.00, 860.00, 871.50, 881.50, 879.50, 855.50,
    876.50, 866.00, 859.50, 868.50, 951.00, 867.00, 873.00, 865.50, 856.50};
// 26,27,29,13
double maxRecordedValue[LDRPINCOUNT]{
    881, 882, 863, 870, 874, 864, 857, 871, 868, 862, 861, 873,
    866, 871, 879, 872, 870, 890, 881, 873, 869, 881, 869, 883,
    891, 891, 864, 887, 876, 869, 877, 952, 874, 880, 876, 866};

double minRecordedValue[LDRPINCOUNT]{
    859, 865, 835, 854, 846, 836, 839, 844, 850, 836, 833, 840,
    857, 850, 857, 858, 864, 868, 864, 853, 854, 857, 851, 860,
    872, 868, 847, 866, 856, 850, 860, 950, 860, 866, 855, 847};

double calculatedthesholdValue[LDRPINCOUNT]{
    834, 833, 818, 822, 828, 821, 826, 820, 5,   821, 832, 822,
    830, 832, 825, 841, 825, 832, 838, 834, 829, 841, 843, 840,
    850, 846, 848, 843, 836, 840, 830, 831, 828, 844, 823, 827};
double highValues[LDRPINCOUNT] = {
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00};

};

struct CameraTxData {
    double values[6];
};

struct CameraTxPayload {
    CameraTxData cameraTxData;
};

struct L1TxData {
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