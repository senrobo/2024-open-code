#ifndef CONFIG_H
#define CONFIG_H

/*
------------------------------------------------------------------------------------
SETTINGS
------------------------------------------------------------------------------------
*/

#define ROBOT1
#define TEST_DEFENCE_ROBOT
#define NORMAL_CODE
//#define LOCALISE_CODE

#define DEBUG_EVERYTHING
// #define DEBUG_LIGHT_RING
// #define DEBUG_THRESHOLD_VALUES
//  #define DEBUG_MOVEMENT
//  #define DEBUG_DEFENCE_BOT
//  #define DEBUG_LIDAR

#ifdef ROBOT2
    #define CATCHMENT_THRESHOLD 85
#endif
#ifdef ROBOT1
    #define CATCHMENT_THRESHOLD 280 // 280
#endif

/*
------------------------------------------------------------------------------------
PINOUTS/SETTINGS
------------------------------------------------------------------------------------
*/

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

// dribbler settings
#define DRIBBLER_PWM_PIN        0
#define DRIBBLER_LOWER_LIMIT    128
#define DRIBBLER_UPPER_LIMIT    230
#define BRUSHLESS_DEFAULT_SPEED 155 // 220
#define BRUSHLESS_FAST_SPEED    155

// all delays
#define CATCH_BALL_DELAY_TIME    500 // in millis
#define MIN_KICK_TIME            2000
#define DRIBBLER_KICK_DELAY_TIME 500

/*
------------------------------------------------------------------------------------
DEFENCE ROBOT
Instructions:
1. If u are setting it at defence bot manually, pls uncomment TEST_DEFENCE_ROBOT

-------------------------------------------------------------------------------------
*/

#define DEFENCE_STOP_LINE_TRACK_LIDAR_DIST                                     \
    0 // invalidated for now due to unreliability

// Bearing PID
#define MIN_BEARING_CORRECTION   -500 // max motor speed in negative bearing
#define MAX_BEARING_CORRECTION   500  // max motor speed in positive bearing
#define DEFENCE_ROBOT_BEARING_KP 5
#define DEFENCE_ROBOT_BEARING_KD 0.05
#define DEFENCE_ROBOT_BEARING_KI                                               \
    0 // dont touch this unless u know what u are doing

// Linetrack PID
#define MIN_LINETRACK_CORRECTION                                               \
    -0.3 // max offset direction in negative direction
#define MAX_LINETRACK_CORRECTION                                               \
    0.3 // max offset direction in positive direction
#define DEFENCE_ROBOT_LINETRACK_KP 0.5
#define DEFENCE_ROBOT_LINETRACK_KD 0.05
#define DEFENCE_ROBOT_LINETRACK_KI                                             \
    0 // dont touch this unless u know what u are doing
#define DEFENCE_DEPTH_IN_LINE 0.8 // from 0 to 2

// REJECTION USING CAMERA LOCALISATION

// WHEN ON LINE ------------------------------------
#define DEFENCE_Y_AXIS_REJECTION -85 // this condition must be fulfilled // -80
#define DEFENCE_MIN_X_AXIS_REJECECTION                                         \
    -130                               // either min or max must be fulfiiled // -13
#define DEFENCE_MAX_X_AXIS_REJECECTION 13 // either min or max must be fulfiiled // 13
#define DEFENCE_REJECTION_VELOCITY     250
// When Tracking Ball--------------------------------------------------
#define DEFENCE_TRACKBALL_MAX_VELOCITY                                         \
    800 // this is a decelerration code from MAX to MIN
#define DEFENCE_TRACKBALL_MIN_VELOCITY 0
// https://www.desmos.com/calculator/btpwnfdqyq
#define DEFENCE_ACCELERATION_MULTIPLIER                                        \
    4 // larger the value, the acceleration is slower(speeds up or slows down
      // slower)
// WHEN NO BALL IS DETECTED, CENTRE IN MIDDLE -------------------------------
#define DEFENCE_NO_BALL_VELOCITY 250

/*
------------------------------------------------------------------------------------
ATTACK ROBOT
Instructions:
DONT TOUCH WITHOUT ASKING ME
-------------------------------------------------------------------------------------
*/

// curve around ball settings
// https://www.desmos.com/calculator/atvzoxtoxn
#define OFFSET_MULTIPLIER 0.04F // variable a in desmos
#define START_OFFSET      28    // variable d
#define DEGREE_MULTIPLIER 20

#define KICK_BEARING_ERROR             6
#define X_LOCALISATION_ERROR_THRESHOLD 10
#define Y_LOCALISATION_ERROR_THRESHOLD 10

// strategy 2
#define X_BALL_STRATEGY2_RANGE 30000000  // 30
#define Y_BALL_STRATEGY2       -70000000 //-70
#define MIN_TURN_AROUND_TIME   5000

/*
------------------------------------------------------------------------------------
IMPORTANT POSITIONS
-------------------------------------------------------------------------------------
*/
#define DEFAULT_POSITION                                                       \
    (Point) { 30, 30 }

/*
------------------------------------------------------------------------------------
SLOWDOWN AT BOUNDARIES (ONLY APPLICABLE FOR ATTACK CODE)
-------------------------------------------------------------------------------------
*/

// slowdown with localisation code
#define X_AXIS_SLOWDOWN_START          45// 45
#define X_AXIS_SLOWDOWN_END            80
#define X_NEGATIVE_AXIS_SLOWDOWN_START -45 //-45
#define X_NEGATIVE_AXIS_SLOWDOWN_END   -80
#define X_AXIS_SLOWDOWN_SPEED          0.8F

// For goal
#define Y_AXIS_SLOWDOWN_SPEED_GOAL          0.8F
#define Y_AXIS_SLOWDOWN_START_GOAL          60// 45
#define Y_AXIS_SLOWDOWN_END_GOAL            90
#define Y_NEGATIVE_AXIS_SLOWDOWN_START_GOAL -60 //-5
#define Y_NEGATIVE_AXIS_SLOWDOWN_END_GOAL   -90

#define Y_AXIS_SLOWDOWN_SPEED_EDGE 0.9F
#define Y_AXIS_SLOWDOWN_START_EDGE 70
#define Y_AXIS_SLOWDOWN_END_EDGE   110
#define Y_NEGATIVE_AXIS_SLOWDOWN_START_EDGE -70 
#define Y_NEGATIVE_AXIS_SLOWDOWN_END_EDGE   -110

// DEFENCE BOT
//  slowdown with localisation code
#define X_DEFENCE_AXIS_SLOWDOWN_START          45 // 45
#define X_DEFENCE_AXIS_SLOWDOWN_END            80
#define X_DEFENCE_NEGATIVE_AXIS_SLOWDOWN_START -45 //-45
#define X_DEFENCE_NEGATIVE_AXIS_SLOWDOWN_END   -80
#define X_DEFENCE_AXIS_SLOWDOWN_SPEED          0.8F

// For goal
#define Y_DEFENCE_AXIS_SLOWDOWN_SPEED_GOAL  0.5F
#define Y_DEFENCE_AXIS_SLOWDOWN_START_GOAL  60 // 45
#define Y_DEFENCE_AXIS_SLOWDOWN_END_GOAL            90
#define Y_DEFENCE_NEGATIVE_AXIS_SLOWDOWN_START_GOAL -78 //-5
#define Y_DEFENCE_NEGATIVE_AXIS_SLOWDOWN_END_GOAL   -83

#define X_GOAL_WIDTH 30

/*
------------------------------------------------------------------------------------
ADVANCED
-------------------------------------------------------------------------------------
*/

// movement
#define ACCELERATION_ERROR 20.0

#define FLICK_STRATEGY
// #define TURN_AND_KICK_STRATEGY

/*
------------------------------------------------------------------------------------
OTHERS
-------------------------------------------------------------------------------------
*/

#define LOCALISE_CODE_TARGET_BEARING 0 // do this

#endif
