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
        870.60, 877.00, 855.00, 866.90, 856.40, 849.00, 850.40, 854.10, 863.30,
        850.10, 849.60, 854.80, 866.60, 860.00, 864.10, 867.90, 865.80, 873.30,
        869.80, 863.40, 863.00, 866.70, 866.60, 869.10, 882.50, 876.00, 860.00,
        874.20, 866.60, 866.80, 877.90, 955.00, 873.50, 874.60, 869.20, 860.30};
    // 26,27,29,13
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
        827.80, 829.10, 815.20, 831.00, 836.40, 819.80, 842.10, 831.80, 819.20,
        838.80, 844.80, 831.80, 841.20, 841.30, 833.10, 851.40, 824.10, 824.50,
        830.20, 824.10, 823.00, 826.40, 834.50, 827.70, 838.20, 831.40, 836.50,
        826.40, 832.00, 836.50, 824.40, 822.00, 828.40, 841.80, 820.30, 825.00};
    // 26,27,29,13
    double maxRecordedValue[LDRPINCOUNT]{
        846, 848, 839, 852, 856, 845, 861, 850, 843, 857, 863, 850,
        858, 856, 852, 864, 843, 842, 847, 843, 844, 846, 852, 848,
        855, 851, 854, 846, 853, 854, 844, 843, 848, 860, 842, 846};

    double minRecordedValue[LDRPINCOUNT]{
        820, 821, 805, 822, 828, 809, 834, 824, 809, 831, 837, 824,
        834, 835, 825, 846, 816, 817, 823, 816, 814, 818, 827, 819,
        831, 823, 829, 818, 823, 829, 816, 813, 820, 834, 811, 816};

    double calculatedthesholdValue[LDRPINCOUNT]{
        834, 833, 818, 822, 828, 821, 826, 820, 5,   821, 832, 822,
        830, 832, 825, 841, 825, 832, 838, 834, 829, 841, 843, 840,
        850, 846, 848, 843, 836, 840, 830, 831, 828, 844, 823, 827};
    double highValues[LDRPINCOUNT] = {
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00};
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
    int setStrategy = 1;
    int kickStrategySequence = 0;
    int kickComplete = 0;
    int lastavoidTime = 0;
    bool setAvoidBotDirection = true;
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

// lightring
void selectMUXChannel(uint8_t channel);
int readMUXChannel(int index, int muxInput);
void getValues();
void findLine();

// functions
#endif