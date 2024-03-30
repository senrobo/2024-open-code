#ifndef MAIN_H
#define MAIN_H

#include "ArduinoEigen.h"
#include "config.h"
#include "movement.h"
#include "shared.h"
#include "util.h"

#define LDRPINCOUNT 36

#ifdef ROBOT1
struct LightArray {
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
        320.00, 330.00, 340.00, 350.00}; // assuming placement of ldrs is
                                         // constant
    float LDRThresholds[LDRPINCOUNT]{
        862.00, 864.50, 846.00, 857.50, 849.00, 845.00, 844.50, 848.50, 856.50,
        845.50, 845.00, 850.00, 859.00, 852.50, 857.50, 859.00, 857.00, 863.00,
        859.50, 854.00, 854.00, 857.00, 854.50, 857.00, 869.50, 867.00, 850.50,
        862.50, 857.00, 855.50, 865.50, 934.00, 857.50, 859.50, 857.50, 854.50};
    double maxRecordedValue[LDRPINCOUNT]{
        886, 891, 876, 883, 876, 870, 870, 873, 878, 869, 872, 873,
        882, 881, 883, 884, 884, 888, 888, 883, 884, 887, 889, 888,
        900, 897, 881, 891, 889, 885, 894, 955, 891, 890, 886, 882};

    double minRecordedValue[LDRPINCOUNT]{
        864, 871, 846, 860, 848, 840, 842, 846, 857, 842, 840, 847,
        860, 851, 856, 861, 858, 867, 862, 855, 854, 858, 857, 861,
        875, 867, 851, 867, 857, 859, 871, 955, 866, 868, 862, 851};

    double calculatedthesholdValue[LDRPINCOUNT]{
        834, 833, 818, 822, 828, 821, 826, 820, 5,   821, 832, 822,
        830, 832, 825, 841, 825, 832, 838, 834, 829, 841, 843, 840,
        850, 846, 848, 843, 836, 840, 830, 831, 828, 844, 823, 827};
    double highValues[LDRPINCOUNT] = {
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00};
    long long combinedLDRValues[36] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    long long averageLDRValuesTime = 0;
    long long averageLDRCounter = 0;
    long long averageLDRValues[36] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
};

#endif

#ifdef ROBOT2
struct LightArray {
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
        320.00, 330.00, 340.00, 350.00}; // assuming placement of ldrs is
                                         // constant

    float c = 10;
    float LDRThresholds[LDRPINCOUNT]{
        862.00, 864.50, 846.00, 857.50, 849.00, 845.00, 844.50, 848.50, 856.50,
        845.50, 845.00, 850.00, 859.00, 852.50, 857.50, 859.00, 857.00, 863.00,
        859.50, 854.00, 854.00, 857.00, 854.50, 857.00, 869.50, 867.00, 850.50,
        862.50, 857.00, 855.50, 865.50, 934.00, 857.50, 859.50, 857.50, 854.50};
    // 26,27,29,13
    double maxRecordedValue[LDRPINCOUNT]{
        869, 872, 857, 867, 860, 857, 855, 859, 865, 857, 856, 860,
        867, 862, 866, 868, 867, 871, 869, 865, 865, 869, 866, 868,
        879, 877, 863, 873, 870, 868, 875, 935, 867, 870, 866, 864};

    double minRecordedValue[LDRPINCOUNT]{
        855, 857, 835, 848, 838, 833, 834, 838, 848, 834, 834, 840,
        851, 843, 849, 850, 847, 855, 850, 843, 843, 845, 843, 846,
        860, 857, 838, 852, 844, 843, 856, 933, 848, 849, 849, 845};

    double calculatedthesholdValue[LDRPINCOUNT]{
        834, 833, 818, 822, 828, 821, 826, 820, 5,   821, 832, 822,
        830, 832, 825, 841, 825, 832, 838, 834, 829, 841, 843, 840,
        850, 846, 848, 843, 836, 840, 830, 831, 828, 844, 823, 827};
    double highValues[LDRPINCOUNT] = {
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00};

    long long combinedLDRValues[36] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    long long averageLDRValuesTime = 0;
    long long averageLDRCounter = 0;
    long long averageLDRValues[36] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
};

#endif
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

    int onLine = 0;
    float angleBisector;
    float depthinLine;
    int linetrackldr1;
    int linetrackldr2;
    int ballinCatchment;
};

struct L3sensorValues {
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

struct L3Txpayloaf {
    L3sensorValues sensorValues;
};

struct Solenoid {
    int kick = 5;
};

struct L2TxtoL1payload {
    Solenoid solenoid;
};

struct ProcessedValues {
    Vector ball_relativeposition;
    Vector yellowgoal_relativeposition;
    Vector bluegoal_relativeposition;
    int ballExists = 0;
    int yellowgoal_exists = 0;
    int bluegoal_exists = 0;
    int lidarDistance[4];
    double lidarConfidence[4];
    double relativeBearing;
    Point robot_position;
    double defenceRobotAngleBisector;
    double defenceRobotDepthinLine;
    double averageCatchmentValues;
};

struct TruthValues {
    double bearingtoField;
    Vector ballPositiontoField;
};

struct Execution {
    double targetBearing = 0;
    int strategy = 1;
    int kickStrategySequence = 0;
    int kickComplete = 0;
    int lastavoidTime = 0;
    bool setAvoidBotDirection = true;
    int catchmentTime = 0;
    int lastkickTime = 0;
    int lastTurnBackwardsTime = 0;
};

// global variables

extern Solenoid solenoid;
extern TruthValues truthValues;
extern Movement movement;
extern ProcessedValues processedValues;
extern SensorValues sensorValues;
extern Execution execution;
extern LightArray lightArray;
// subroutines

Vector localizeWithOffensiveGoal();
Vector localizeWithDefensiveGoal();
Vector localizeWithBothGoals();
double ballAngleOffset(double distance, double direction);
void receiveCameraTxData(const byte *buf, size_t size);
void attachBrushless();
double curveAroundBallMultiplier(double angle, double actual_distance,
                                 double start_distance);
void verifyingObjectExistance();
void processLidars();
void avg_LDRValues(int dt_micros, int delay, int i);

// lightring
void selectMUXChannel(uint8_t channel);
int readMUXChannel(int index, int muxInput);
void getValues();
void findLine(int dt_micros);

// functions
#endif