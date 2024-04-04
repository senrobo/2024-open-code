#include <Arduino.h>

#include <array>

#include "MovingAverage.h"
#include "PacketSerial.h"
#include "config.h"
#include "main.h"
#include "movement.h"
#include "shared.h"
#include "util.h"

// Buffer (and added samples) will be initialised as uint8_t, total 16 samples
MovingAverage<uint8_t, 16> filter;

void selectMUXChannel(uint8_t channel) {
    digitalWrite(S0, channel & 1);
    digitalWrite(S1, (channel >> 1) & 1);
    digitalWrite(S2, (channel >> 2) & 1);
    digitalWrite(S3, (channel >> 3) & 1);
}
int readMUXChannel(int index, int muxInput) {
    selectMUXChannel(index);
    return analogRead(muxInput);
}
void getValues() {
    // values from
    // https://docs.google.com/spreadsheets/d/14BAutKBhzy3ZVvEh6iIaexv4C6UqhUbg-NXU7_EWB44/edit#gid=0

    lightArray.RAWLDRVALUES[0] = readMUXChannel(2, MuxInput2);
    lightArray.RAWLDRVALUES[1] = readMUXChannel(1, MuxInput2);
    lightArray.RAWLDRVALUES[2] = readMUXChannel(0, MuxInput2);
    lightArray.RAWLDRVALUES[3] = readMUXChannel(7, MuxInput1);
    lightArray.RAWLDRVALUES[4] = readMUXChannel(6, MuxInput1);
    lightArray.RAWLDRVALUES[5] = readMUXChannel(5, MuxInput1);
    lightArray.RAWLDRVALUES[6] = readMUXChannel(4, MuxInput1);
    lightArray.RAWLDRVALUES[7] = readMUXChannel(3, MuxInput1);
    lightArray.RAWLDRVALUES[8] = readMUXChannel(2, MuxInput1);
    lightArray.RAWLDRVALUES[9] = readMUXChannel(1, MuxInput1);
    lightArray.RAWLDRVALUES[10] = readMUXChannel(0, MuxInput1);
    lightArray.RAWLDRVALUES[11] = readMUXChannel(15, MuxInput1);
    lightArray.RAWLDRVALUES[12] = readMUXChannel(14, MuxInput1);
    lightArray.RAWLDRVALUES[13] = readMUXChannel(13, MuxInput1);
    lightArray.RAWLDRVALUES[14] = readMUXChannel(12, MuxInput1);
    lightArray.RAWLDRVALUES[15] = readMUXChannel(11, MuxInput1);
    lightArray.RAWLDRVALUES[16] = readMUXChannel(15, MuxInput2);
    lightArray.RAWLDRVALUES[17] = readMUXChannel(14, MuxInput2);
    lightArray.RAWLDRVALUES[18] = readMUXChannel(13, MuxInput2);
    lightArray.RAWLDRVALUES[19] = readMUXChannel(12, MuxInput2);
    lightArray.RAWLDRVALUES[20] = readMUXChannel(11, MuxInput2);
    lightArray.RAWLDRVALUES[21] = readMUXChannel(12, MuxInput3);
    lightArray.RAWLDRVALUES[22] = readMUXChannel(11, MuxInput3);
    lightArray.RAWLDRVALUES[23] = readMUXChannel(10, MuxInput3);
    lightArray.RAWLDRVALUES[24] = readMUXChannel(9, MuxInput3);
    lightArray.RAWLDRVALUES[25] = readMUXChannel(8, MuxInput3);
    lightArray.RAWLDRVALUES[26] = readMUXChannel(7, MuxInput3);
    lightArray.RAWLDRVALUES[27] = readMUXChannel(6, MuxInput3);
    lightArray.RAWLDRVALUES[28] = readMUXChannel(5, MuxInput3);
    lightArray.RAWLDRVALUES[29] = readMUXChannel(4, MuxInput3);
    lightArray.RAWLDRVALUES[30] = readMUXChannel(3, MuxInput3);
    lightArray.RAWLDRVALUES[31] = readMUXChannel(2, MuxInput3);
    lightArray.RAWLDRVALUES[32] = readMUXChannel(1, MuxInput3);
    lightArray.RAWLDRVALUES[33] = readMUXChannel(0, MuxInput3);
    lightArray.RAWLDRVALUES[34] = readMUXChannel(4, MuxInput2);
    lightArray.RAWLDRVALUES[35] = readMUXChannel(3, MuxInput2);
    // ball catchment ldr
    sensorValues.ballinCatchment = readMUXChannel(5, MuxInput2);
}

void avg_LDRValues(int dt_micros, int delay) {
    lightArray.averageLDRValuesTime += dt_micros;
    if (lightArray.averageLDRValuesTime < delay) {
        for (int i = 0; i < 36; i++) {
            lightArray.combinedLDRValues[i] += lightArray.RAWLDRVALUES[i];
            if (i == 35) { lightArray.averageLDRCounter++; }
        }
    } else {
        for (int i = 0; i < 36; i++) {

            lightArray.averageLDRValues[i] =
                lightArray.combinedLDRValues[i] / lightArray.averageLDRCounter;

            lightArray.combinedLDRValues[i] = 0;

            if (i == 35) {
                lightArray.averageLDRCounter = 0;
                lightArray.averageLDRValuesTime = 0;
            }
        }
    }
}

void findLine(int dt_micros) {
    getValues();
    // initialize essential variables
    int first_tmpldrangle = 0;
    int first_ldrPinout = 0;
    int second_tmpldrangle = 0;
    int second_ldrPinout = 0;
    double tmpanglediff = 0;
    double largestanglediff = 0;
    double tmprobotangle = 0;

    int final_ldrPinout1 = 0;
    int final_ldrPinout2 = 0;
    sensorValues.onLine = 1;
    avg_LDRValues(dt_micros, 20000);

    for (int pinNumber = 0; pinNumber < LDRPINCOUNT; pinNumber++) {
        if (lightArray.highValues[pinNumber] <
            lightArray.averageLDRValues[pinNumber]) {
            lightArray.highValues[pinNumber] =
                lightArray.averageLDRValues[pinNumber];
        }

#ifdef DEBUG_LIGHT_RING
        // lightArray.highValues[sensorValues.linetrackldr2] =
        //     sensorValues.linetrackldr1;
        const auto printSerial = [](int value) { Serial.printf("%3d", value); };
        if (pinNumber == 35) {
            printSerial(lightArray.highValues[pinNumber]);
            Serial.println(" ");
        } else {
            printSerial(lightArray.highValues[pinNumber]);
            Serial.print(" , ");
        }
#endif
        if (lightArray.averageLDRValues[pinNumber] >
            lightArray.LDRThresholds[pinNumber]) {
            sensorValues.onLine = 2;
            first_ldrPinout = pinNumber;
            first_tmpldrangle = lightArray.LDRBearings[pinNumber];

            for (int i = 0; i < LDRPINCOUNT; i++) {
                if (lightArray.averageLDRValues[i] >
                    lightArray.LDRThresholds[i]) {
                    second_ldrPinout = i;
                    second_tmpldrangle = lightArray.LDRBearings[i];
                    tmpanglediff =
                        abs(lightArray.LDRBearings[second_ldrPinout] -
                            lightArray.LDRBearings[first_ldrPinout]);

                    if (tmpanglediff > 180) tmpanglediff = 360 - tmpanglediff;
                    if (tmpanglediff > largestanglediff) {
                        //(first_tmpldrangle > 180) ? tmprobotangle = 360 -
                        // first_tmpldrangle : first_tmpldrangle;
                        largestanglediff = tmpanglediff;
                        final_ldrPinout1 = first_ldrPinout;
                        final_ldrPinout2 = second_ldrPinout;
                    }
                }
            }
        }

        sensorValues.linetrackldr1 = final_ldrPinout1;
        sensorValues.linetrackldr2 = final_ldrPinout2;
        if (largestanglediff > 180) largestanglediff = 360 - largestanglediff;
        if (abs(lightArray.LDRBearings[final_ldrPinout2] -
                lightArray.LDRBearings[final_ldrPinout1]) <= 180) {
            sensorValues.angleBisector =
                lightArray.LDRBearings[final_ldrPinout1] + largestanglediff / 2;
        } else {
            sensorValues.angleBisector =
                lightArray.LDRBearings[final_ldrPinout2] + largestanglediff / 2;
        }
        sensorValues.angleBisector =
            clipAngleto360degrees(360 - sensorValues.angleBisector);
        sensorValues.depthinLine =
            1.0 - cosf((largestanglediff / 2.0) / 180.0 * PI);
        //(1.0 - (cosf((90/2)/180 * PI) / 1.0));
    }
}

double ballAngleOffset(double distance, double direction) {
    // offset multiplier https://www.desmos.com/calculator/8d2ztl2zf8

    if (direction < 50) {
        double constant = 0;
        double angleoffset =
            constrain(direction * 1.5, -90, 90) *
            fmin(powf(exp(1), OFFSET_MULTIPLIER * (START_OFFSET - distance)),
                 1);
        return angleoffset;
    } else {
        double angleoffset =
            constrain(direction * 1, -90, 90) *
            fmin(powf(exp(1), OFFSET_MULTIPLIER * (START_OFFSET - distance)),
                 1);
        return angleoffset;
    }
};

double curveAroundBallMultiplier(double angle, double actual_distance,
                                 double start_distance) {
    double radial_distance =
        sqrtf(powf(angle, 2)) / 180 * DEGREE_MULTIPLIER * PI;
    // Serial.print(radial_distance);
    // Serial.print(", ");
    return (radial_distance + actual_distance) / start_distance;
}

void attachBrushless() {

#ifdef ROBOT2
    analogWriteFrequency(DRIBBLER_PWM_PIN, 1000); // 4096

    analogWrite(DRIBBLER_PWM_PIN, DRIBBLER_LOWER_LIMIT);
    delay(3000);
    analogWrite(DRIBBLER_PWM_PIN, DRIBBLER_UPPER_LIMIT);
    delay(100);
#endif
#ifdef ROBOT1
    analogWriteFrequency(DRIBBLER_PWM_PIN, 1000); // 4096

    analogWrite(DRIBBLER_PWM_PIN, DRIBBLER_LOWER_LIMIT);
    delay(3000);
    analogWrite(DRIBBLER_PWM_PIN, DRIBBLER_UPPER_LIMIT);
    delay(100);
#endif
}

// Goal Localisation
// Compute the localized position using the offensive goal vector
// Takes sensor input of the yellow goal's relative position and the bearing to
// the field Calculates a "fake" center vector by adding the yellow goal's
// actual position to a predefined vector Adjusts the angle of the fake center
// vector and returns the actual center vector

// Lidar Processing
// Process the lidar values to get the relative position of the robot to the
// field Takes the lidar values and the bearing to the field Calculates the
// relative position of the robot to the field Adjusts the confidence of the
// lidar values and stores them in the processedValues struct
