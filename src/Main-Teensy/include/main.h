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
        806.00, 815.60, 788.30, 804.30, 796.30, 792.50, 795.30, 806.00, 816.60,
        798.10, 797.20, 804.80, 813.00, 804.30, 813.00, 795.80, 798.40, 804.20,
        789.10, 780.40, 794.10, 799.90, 793.40, 763.30, 823.20, 821.00, 780.90,
        802.00, 808.40, 811.00, 825.50, 933.00, 802.80, 797.20, 804.20, 785.90};
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
      708.90, 713.30, 698.90, 697.20, 701.60, 366.70, 709.30, 714.00, 649.60,
        714.30, 713.70, 625.90, 663.90, 658.00, 720.10, 723.00, 689.90, 708.20,
        652.10, 666.80, 675.30, 599.10, 625.20, 713.70, 726.80, 721.20, 724.00,
        717.90, 717.50, 731.10, 712.40, 704.40, 699.30, 721.70, 654.60, 661.20};
    // 26,27,29,13
    double maxRecordedValue[LDRPINCOUNT]{
        823, 826, 812, 813, 817, 807, 820, 817, 809, 816, 822, 743,
        815, 812, 817, 833, 818, 819, 826, 819, 819, 822, 832, 824,
        833, 831, 835, 825, 823, 829, 822, 820, 817, 830, 816, 821};

    double minRecordedValue[LDRPINCOUNT]{
        761, 767, 752, 739, 749, 684, 768, 771, 716, 771, 772, 673,
        708, 698, 770, 789, 767, 774, 764, 757, 758, 766, 788, 781,
        791, 788, 792, 780, 777, 786, 773, 769, 761, 783, 763, 762};

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
    double attackRobotAngleBisector;
    double attackRobotDepthinLine;
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
    int lastdribblerKickDelayTime = 0;
    int lastLineRejectTime = 0;
    int strategy2Quadrant = 0;
    bool setAvoidBotDirection = true;
    int catchmentTime = 0;
    int lastkickTime = 0;
    int lastTurnBackwardsTime = 0;
    int lastTurnandKickTime = 0;
    double direction = 0.0;
    double dribblerSpeed = 0;
    double dribblerSpeedAccel = 140;
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