#ifndef MAIN_H
#define MAIN_H

#include "config.h"
#include "movement.h"
#include "shared.h"
#include "util.h"

#define LDRPINCOUNT 36

#ifdef ROBOT1
struct LightArray {
    double RAWLDRVALUES[LDRPINCOUNT] = {
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
        830.00, 837.20, 814.10, 830.10, 822.10, 819.50, 821.10, 830.00, 838.20,
        822.70, 822.40, 827.60, 837.00, 830.10, 837.00, 830.60, 830.80, 835.40,
        825.70, 818.80, 824.70, 829.30, 825.80, 819.10, 848.40, 845.00, 816.30,
        832.00, 834.80, 835.00, 846.50, 933.00, 831.60, 828.40, 829.40, 815.30};
    double maxRecordedValue[LDRPINCOUNT]{
        842, 848, 827, 843, 835, 833, 834, 842, 849, 835, 835, 839,
        849, 843, 849, 848, 847, 851, 844, 838, 840, 844, 842, 847,
        861, 857, 834, 847, 848, 847, 857, 933, 846, 844, 842, 830};

    double minRecordedValue[LDRPINCOUNT]{
        802, 812, 784, 800, 792, 788, 791, 802, 813, 794, 793, 801,
        809, 800, 809, 790, 793, 799, 783, 774, 789, 795, 788, 754,
        819, 817, 775, 797, 804, 807, 822, 933, 798, 792, 800, 781};

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
        853.00, 855.00, 844.50, 845.50, 848.00, 836.50, 847.00, 844.00, 835.50,
        843.00, 848.50, 767.00, 840.00, 837.00, 842.50, 858.50, 845.50, 850.50,
        856.50, 852.00, 850.00, 855.50, 859.50, 854.00, 865.50, 860.00, 866.00,
        856.50, 855.50, 858.00, 852.50, 849.00, 850.50, 860.50, 848.00, 852.00};
    // 26,27,29,13
    double maxRecordedValue[LDRPINCOUNT]{
        859, 861, 853, 853, 856, 846, 855, 852, 843, 850, 856, 773,
        847, 845, 851, 864, 853, 858, 863, 859, 858, 864, 867, 863,
        872, 868, 873, 864, 863, 865, 861, 857, 858, 866, 856, 859};

    double minRecordedValue[LDRPINCOUNT]{
        847, 849, 836, 838, 840, 827, 839, 836, 828, 836, 841, 761,
        833, 829, 834, 853, 838, 843, 850, 845, 842, 847, 852, 845,
        859, 852, 859, 849, 848, 851, 844, 841, 843, 855, 840, 845};

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
    int attackMode = 1; 
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
    double averageballExists;
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
    double direction = 0.0;
    int dribblerSpeed = 0;
    int attackMode = 1;
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