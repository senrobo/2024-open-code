#ifndef CONFIG_H
#define CONFIG_H



#define S0           15
#define S1           16
#define S2           17
#define S3           18
#define MuxInput1    19
#define MuxInput2    22
#define MuxInput3    23
#define Solenoid_Pin 20


#define LDRPINCOUNT 36
#define RadiusofLDR 1.0F

// Set Code
#define LDRPINCOUNT 36
#define RadiusofLDR 1.0F



static int RAWLDRVALUES[LDRPINCOUNT] = {
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00};
static int LDRPINMap[LDRPINCOUNT]{0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
                           12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24,
                           25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};

static float LDRBearings[LDRPINCOUNT]{
    0.00,   10.00,  20.00,  30.00,  40.00,  50.00,  60.00,  70.00,
    80.00,  90.00,  100.00, 110.00, 120.00, 130.00, 140.00, 150.00,
    160.00, 170.00, 180.00, 190.00, 200.00, 210.00, 220.00, 230.00,
    240.00, 250.00, 260.00, 270.00, 280.00, 290.00, 300.00, 310.00,
    320.00, 330.00, 340.00, 350.00}; // assuming placement of ldrs is constant

static float LDRThresholds[LDRPINCOUNT]{
    870.00, 873.50, 849.00, 862.00, 860.00, 850.00, 848.00, 857.50, 859.00,
    849.00, 847.00, 856.50, 861.50, 860.50, 868.00, 865.00, 867.00, 879.00,
    872.50, 863.00, 861.50, 869.00, 860.00, 871.50, 881.50, 879.50, 855.50,
    876.50, 866.00, 859.50, 868.50, 951.00, 867.00, 873.00, 865.50, 856.50};
// 26,27,29,13
static double maxRecordedValue[LDRPINCOUNT]{
    881, 882, 863, 870, 874, 864, 857, 871, 868, 862, 861, 873,
    866, 871, 879, 872, 870, 890, 881, 873, 869, 881, 869, 883,
    891, 891, 864, 887, 876, 869, 877, 952, 874, 880, 876, 866};

static double minRecordedValue[LDRPINCOUNT]{
    859, 865, 835, 854, 846, 836, 839, 844, 850, 836, 833, 840,
    857, 850, 857, 858, 864, 868, 864, 853, 854, 857, 851, 860,
    872, 868, 847, 866, 856, 850, 860, 950, 860, 866, 855, 847};

static double calculatedthesholdValue[LDRPINCOUNT]{
    834, 833, 818, 822, 828, 821, 826, 820, 5,   821, 832, 822,
    830, 832, 825, 841, 825, 832, 838, 834, 829, 841, 843, 840,
    850, 846, 848, 843, 836, 840, 830, 831, 828, 844, 823, 827};
static double highValues[LDRPINCOUNT] = {
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00};


//#define DEBUG_EVERYTHING
#define DEBUG_LIGHT_RING
// #define DEBUG_THRESHOLD_VALUES
// #define DEBUG_MOVEMENT

// #define DEFENCE_BOT_CODE
// print Serial
//#define DEBUG_DEFENCE_BOT
// defence Bot settings
#define DEFENCE_DEPTH_IN_LINE 0.7 // from 0 to 2
// https://www.desmos.com/calculator/btpwnfdqyq
#define DEFENCE_ACCELERATION_MULTIPLIER                                        \
    2 // smaller the value, the acceleration is slower(speeds up or slows down
      // slower)

#define DEFENCE_STOP_LINE_TRACK_LIDAR_DIST 40

#define ATTACK_BOT_CODE
// stealth strategy
#define KICK_BEARING_ERROR             20
#define X_LOCALISATION_ERROR_THRESHOLD 20
#define Y_LOCALISATION_ERROR_THRESHOLD 20

// curve around ball settings
// https://www.desmos.com/calculator/atvzoxtoxn
#define OFFSET_MULTIPLIER 0.02F // variable a in desmos
#define START_OFFSET      26    // variable d
#define DEGREE_MULTIPLIER 20

#define DRIBBLER_LOWER_LIMIT 128
#define DRIBBLER_UPPER_LIMIT 230

// #define DEBUG_LIDAR

// #define LOCALISE_CODE
#define LOCALISE_CODE_TARGET_BEARING 0 // do this

// slowdown with localisation code
#define X_AXIS_SLOWDOWN_START 60
#define X_AXIS_SLOWDOWN_END 65
#define Y_AXIS_SLOWDOWN_START 60
#define Y_AXIS_SLOWDOWN_END 70
#define X_AXIS_SLOWDOWN_SPEED 300
#define Y_AXIS_SLOWDOWN_SPEED 300

// dribbler settings
#define DRIBBLER_PWM_PIN        0
#define BRUSHLESS_DEFAULT_SPEED 210 //220

// lidars processing settings
#define X_AXIS_LIDARS_POSITIONAL_OFFSET                                        \
    6.865 // from front of lidar to centre of bot
#define Y_AXIS_LIDARS_POSITIONAL_OFFSET 3.7
#define LIDARSMEASUREMENTOFFSET         2 // from wall to front of lidar
#define WIDTH_OF_FIELD                  182.0F
#define LENGTH_OF_FIELD                 243.0F
#define LENGTH_ERROR                    15.0F
#define WIDTH_ERROR                     10.0F
#define X_CAMERA_ERROR                  8.0F
#define Y_CAMERA_ERROR                  12.0F

#endif