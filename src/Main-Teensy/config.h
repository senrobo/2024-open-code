#ifndef CONFIG_H
#define CONFIG_H

#define S0           15
#define S1           16
#define S2           17
#define S3           18
#define MuxInput1    19
#define MuxInput2    22
#define MuxInput3    23
#define Solenoid_Pin 14

#define LDRPINCOUNT 36
#define RadiusofLDR 1.0F

#define STRATEGY1

#define DEBUG_EVERYTHING
// #define DEBUG_LIGHT_RING
// #define DEBUG_THRESHOLD_VALUES
//  #define DEBUG_MOVEMENT

#define DEFENCE_BOT_CODE
// print Serial
// #define DEBUG_DEFENCE_BOT
// defence Bot settings
#define DEFENCE_DEPTH_IN_LINE 1 // from 0 to 2
// https://www.desmos.com/calculator/btpwnfdqyq
#define DEFENCE_ACCELERATION_MULTIPLIER                                        \
    1.8 // smaller the value, the acceleration is slower(speeds up or slows down
        // slower)

#define DEFENCE_STOP_LINE_TRACK_LIDAR_DIST 40

// #define ATTACK_BOT_CODE
//  stealth strategy
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
#define X_AXIS_SLOWDOWN_END   65
#define Y_AXIS_SLOWDOWN_START 60
#define Y_AXIS_SLOWDOWN_END   70
#define X_AXIS_SLOWDOWN_SPEED 300
#define Y_AXIS_SLOWDOWN_SPEED 300

// dribbler settings
#define DRIBBLER_PWM_PIN        0
#define BRUSHLESS_DEFAULT_SPEED 0 // 220

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