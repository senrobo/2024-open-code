#include <Arduino.h>
#include <ArduinoEigenDense.h>

#include <array>

#include "PacketSerial.h"
#include "config.h"
#include "kalman.h"
#include "main.h"
#include "movement.h"
#include "shared.h"
#include "util.h"


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

void findLine() {
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

    for (int pinNumber = 0; pinNumber < LDRPINCOUNT; pinNumber++) {

#ifdef DEBUG_LIGHT_RING
        const auto printSerial = [](int value) { Serial.printf("%3d", value); };
        if (pinNumber == 35) {
            printSerial(lightArray.RAWLDRVALUES[pinNumber]);
            Serial.println(" ");
        } else {
            printSerial(lightArray.RAWLDRVALUES[pinNumber]);
            Serial.print(" , ");
        }
#endif

        for (int i = 0; i < LDRPINCOUNT; i++) {
            if (lightArray.highValues[i] < lightArray.RAWLDRVALUES[i]) {
                lightArray.highValues[i] = lightArray.RAWLDRVALUES[i];
            }

            if (lightArray.RAWLDRVALUES[pinNumber] > lightArray.LDRThresholds[pinNumber]) {
                sensorValues.onLine = 2;
                first_ldrPinout = pinNumber;
                first_tmpldrangle = lightArray.LDRBearings[pinNumber];

                lightArray.calculatedthesholdValue[i] =
                    lightArray.minRecordedValue[i] +
                    (lightArray.maxRecordedValue[i] - lightArray.minRecordedValue[i]) * 0.5;
                if (lightArray.RAWLDRVALUES[i] > lightArray.LDRThresholds[i]) {
                    second_ldrPinout = i;
                    second_tmpldrangle = lightArray.LDRBearings[i];
                    tmpanglediff = abs(lightArray.LDRBearings[second_ldrPinout] -
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
        sensorValues.depthinLine =
            1.0 - cosf((largestanglediff / 2.0) / 180.0 * PI);
        //(1.0 - (cosf((90/2)/180 * PI) / 1.0));
    }
}

double ballAngleOffset(double distance, double direction) {
    // offset multiplier https://www.desmos.com/calculator/8d2ztl2zf8
    double angleoffset =
        constrain(direction * 1, -90, 90) *
        fmin(powf(exp(1), OFFSET_MULTIPLIER * (START_OFFSET - distance)), 1);
    return angleoffset;
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
    analogWriteFrequency(DRIBBLER_PWM_PIN, 1000);
    analogWrite(DRIBBLER_PWM_PIN, DRIBBLER_LOWER_LIMIT);
    delay(3000);
    analogWrite(DRIBBLER_PWM_PIN, DRIBBLER_UPPER_LIMIT);
    delay(100);
}

void verifyingObjectExistance() {
    processedValues.ballExists =
        (processedValues.ball_relativeposition.distance == 0) ? 0 : 1;
    processedValues.bluegoal_exists =
        (sensorValues.bluegoal_relativeposition.distance == 0) ? 0 : 1;
    processedValues.yellowgoal_exists =
        (sensorValues.yellowgoal_relativeposition.distance == 0) ? 0 : 1;
}

// Goal Localisation
// Compute the localized position using the offensive goal vector
// Takes sensor input of the yellow goal's relative position and the bearing to
// the field Calculates a "fake" center vector by adding the yellow goal's
// actual position to a predefined vector Adjusts the angle of the fake center
// vector and returns the actual center vector

Vector localizeWithOffensiveGoal() {
    const Vector realGoalToCenter = {-180, 113.5};
    const Vector yellow_goalactualposition = {
        sensorValues.yellowgoal_relativeposition.angle -
            truthValues.bearingtoField,
        sensorValues.yellowgoal_relativeposition.distance};
    const auto fakeCenter = yellow_goalactualposition + realGoalToCenter;
    Vector actualCenter = {clipAngleto180degrees(fakeCenter.angle - 180),
                           fakeCenter.distance};
    return actualCenter;
};
Vector localizeWithDefensiveGoal() {
    const Vector realGoalToCenter = {0, 113.5};
    const Vector blue_goalactualposition = {
        sensorValues.bluegoal_relativeposition.angle -
            truthValues.bearingtoField,
        sensorValues.bluegoal_relativeposition.distance};
    const auto fakeCenter = blue_goalactualposition + realGoalToCenter;
    Vector actualCenter = {clipAngleto180degrees(fakeCenter.angle - 180),
                           fakeCenter.distance};
    return actualCenter;
};

Vector localizeWithBothGoals() {
    return (localizeWithDefensiveGoal() + localizeWithOffensiveGoal()) / 2;
};

// Lidar Processing
// Process the lidar values to get the relative position of the robot to the
// field Takes the lidar values and the bearing to the field Calculates the
// relative position of the robot to the field Adjusts the confidence of the
// lidar values and stores them in the processedValues struct

void processLidars() {
    double x_leftrelativetofield = 0;
    double x_rightrelativetofield = 0;
    double y_frontrelativetofield = 0;
    double y_backrelativetofield = 0;
    Point CameraPosition;

    for (int i = 0; i < 4; i++) {
        processedValues.lidarDistance[i] =
            sensorValues.lidardist[i] + LIDARSMEASUREMENTOFFSET;
    }

    if (execution.targetBearing <= 45 && execution.targetBearing > -45) {
        x_leftrelativetofield = (processedValues.lidarDistance[3] +
                                 X_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                cosd(sensorValues.relativeBearing);
        x_rightrelativetofield = (processedValues.lidarDistance[1] +
                                  X_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                 cosd(sensorValues.relativeBearing);
        y_frontrelativetofield = (processedValues.lidarDistance[0] +
                                  Y_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                 cosd(sensorValues.relativeBearing);
        y_backrelativetofield = (processedValues.lidarDistance[2] +
                                 Y_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                cosd(sensorValues.relativeBearing);
    } else if (execution.targetBearing > 45 && execution.targetBearing <= 135) {
        x_leftrelativetofield = (processedValues.lidarDistance[2] +
                                 X_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                cosd(sensorValues.relativeBearing - 90);
        x_rightrelativetofield = (processedValues.lidarDistance[0] +
                                  X_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                 cosd(sensorValues.relativeBearing - 90);
        y_frontrelativetofield = (processedValues.lidarDistance[3] +
                                  Y_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                 cosd(sensorValues.relativeBearing - 90);
        y_backrelativetofield =
            (processedValues.lidarDistance[1] +
             Y_AXIS_LIDARS_POSITIONAL_OFFSET) *
            cosd(clipAngleto180degrees(sensorValues.relativeBearing - 90));
    } else if (execution.targetBearing > 135 ||
               execution.targetBearing <= -135) {
        x_leftrelativetofield =
            (processedValues.lidarDistance[1] +
             X_AXIS_LIDARS_POSITIONAL_OFFSET) *
            cosd(clipAngleto180degrees(sensorValues.relativeBearing - 180));
        x_rightrelativetofield =
            (processedValues.lidarDistance[3] +
             X_AXIS_LIDARS_POSITIONAL_OFFSET) *
            cosd(clipAngleto180degrees(sensorValues.relativeBearing - 180));
        y_frontrelativetofield =
            (processedValues.lidarDistance[2] +
             Y_AXIS_LIDARS_POSITIONAL_OFFSET) *
            cosd(clipAngleto180degrees(sensorValues.relativeBearing - 180));
        y_backrelativetofield =
            (processedValues.lidarDistance[0] +
             Y_AXIS_LIDARS_POSITIONAL_OFFSET) *
            cosd(clipAngleto180degrees(sensorValues.relativeBearing - 180));
    } else if (execution.targetBearing <= -45 &&
               execution.targetBearing > -135) {
        x_leftrelativetofield = (processedValues.lidarDistance[0] +
                                 X_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                cosd(sensorValues.relativeBearing + 90);
        x_rightrelativetofield = (processedValues.lidarDistance[2] +
                                  X_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                 cosd(sensorValues.relativeBearing + 90);
        y_frontrelativetofield = (processedValues.lidarDistance[1] +
                                  Y_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                 cosd(sensorValues.relativeBearing + 90);
        y_backrelativetofield = (processedValues.lidarDistance[3] +
                                 Y_AXIS_LIDARS_POSITIONAL_OFFSET) *
                                cosd(sensorValues.relativeBearing + 90);
    }

    if ((sensorValues.yellowgoal_relativeposition.distance < 90 &&
         processedValues.yellowgoal_exists == 1) ||
        (processedValues.yellowgoal_exists == 1 &&
         processedValues.bluegoal_exists == 0)) {
        CameraPosition = {localizeWithOffensiveGoal().x(),
                          localizeWithOffensiveGoal().y()};
    } else if ((sensorValues.bluegoal_relativeposition.distance < 90 &&
                processedValues.bluegoal_exists == 1) ||
               (processedValues.bluegoal_exists == 1 &&
                processedValues.yellowgoal_exists == 0)) {
        CameraPosition = {localizeWithDefensiveGoal().x(),
                          localizeWithDefensiveGoal().y()};
    } else {
        CameraPosition = {localizeWithBothGoals().x(),
                          localizeWithBothGoals().y()};
    }

    if ((x_leftrelativetofield + x_rightrelativetofield) <
            WIDTH_OF_FIELD - WIDTH_ERROR ||
        (x_leftrelativetofield + x_rightrelativetofield) >
            WIDTH_OF_FIELD + WIDTH_ERROR) {
        processedValues.lidarConfidence[3] =
            (x_leftrelativetofield - (WIDTH_OF_FIELD / 2) >=
                 CameraPosition.x - X_CAMERA_ERROR &&
             x_leftrelativetofield - (WIDTH_OF_FIELD / 2) <=
                 CameraPosition.x + X_CAMERA_ERROR)
                ? 1
                : 0;
        processedValues.lidarConfidence[1] =
            (-x_rightrelativetofield + (WIDTH_OF_FIELD / 2) >=
                 CameraPosition.x - X_CAMERA_ERROR &&
             -x_rightrelativetofield + (WIDTH_OF_FIELD / 2) <=
                 CameraPosition.x + X_CAMERA_ERROR)
                ? 1
                : 0;
    } else {
        processedValues.lidarConfidence[1] = 1;
        processedValues.lidarConfidence[3] = 1;
    }

    if ((y_backrelativetofield + y_frontrelativetofield) <
            LENGTH_OF_FIELD - LENGTH_ERROR ||
        (y_backrelativetofield + y_frontrelativetofield) >
            LENGTH_OF_FIELD + LENGTH_ERROR) {
        processedValues.lidarConfidence[0] =
            (-y_frontrelativetofield + (LENGTH_OF_FIELD / 2) >=
                 CameraPosition.y - Y_CAMERA_ERROR &&
             -y_frontrelativetofield + (LENGTH_OF_FIELD / 2) <=
                 CameraPosition.y + Y_CAMERA_ERROR)
                ? 1
                : 0;
        processedValues.lidarConfidence[2] =
            (y_backrelativetofield - (LENGTH_OF_FIELD / 2) <=
                 CameraPosition.y + Y_CAMERA_ERROR &&
             y_backrelativetofield - (LENGTH_OF_FIELD / 2) >=
                 CameraPosition.y - Y_CAMERA_ERROR)
                ? 1
                : 0;
    } else {
        processedValues.lidarConfidence[0] = 1;
        processedValues.lidarConfidence[2] = 1;
    }
#ifdef DEBUG_PROCESS_LIDARS
    const auto printSerial = [](double value) {
        Serial.printf("%5d", (int)value);
    };
    printSerial(x_leftrelativetofield);
    Serial.print(" | ");
    printSerial(x_rightrelativetofield);
    Serial.print(" | ");
    printSerial(y_frontrelativetofield);
    Serial.print(" | ");
    printSerial(y_backrelativetofield);
    Serial.print(" | ");
    printSerial(sensorValues.relativeBearing);
    Serial.print(" | ");
    printSerial(x_leftrelativetofield + x_rightrelativetofield);
    Serial.print(" | ");
    printSerial(y_backrelativetofield + y_frontrelativetofield);
    Serial.print(" | ");
    printSerial(CameraPosition.x);
    Serial.print(" | ");
    printSerial(CameraPosition.y);
    Serial.print(" | ");
    printSerial(processedValues.lidarConfidence[0]);
    Serial.print(" | ");
    printSerial(processedValues.lidarConfidence[1]);
    Serial.print(" | ");
    printSerial(processedValues.lidarConfidence[2]);
    Serial.print(" | ");
    printSerial(processedValues.lidarConfidence[3]);
    Serial.println(" | ");

#endif
}
