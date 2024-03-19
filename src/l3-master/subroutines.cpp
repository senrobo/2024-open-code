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

double ballAngleOffset(double distance, double direction) {
    // offset multiplier https://www.desmos.com/calculator/8d2ztl2zf8
    double angleoffset =
        constrain(direction * 0.9, -90, 90) *
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
