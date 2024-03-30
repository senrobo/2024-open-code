#ifndef CONFIG_H
#define CONFIG_H

#define YELLOW_GOAL_ATTACK

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