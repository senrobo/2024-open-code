#include <Arduino.h>

#include <array>

#include "PacketSerial.h"
#include "config.h"
#include "main.h"
#include "shared.h"
#include "util.h"

double frontMirrorMapping(double distance) {
    if (distance != 500) {
        return (5.39617219 * powf(10, -7) * powf(distance, 5) -
                1.61827053 * powf(10, -4) * powf(distance, 4) +
                1.88452202 * powf(10, -2) * powf(distance, 3) -
                1.04993865 * powf(10, 0) * powf(distance, 2) +
                2.82679699 * 10 * distance - 
                2.83240269 * powf(10, 2));
    } else
        return 0;
}

double ballMirrorMapping(double distance) {
    if (distance != 500) {
        return (2.72234207 * powf(10, -7) * powf(distance, 5) -
                7.95782065 * powf(10, -5) * powf(distance, 4) +
                9.11543654 * powf(10, -3) * powf(distance, 3) -
                4.96659050 * powf(10, -1) * powf(distance, 2) +
                1.32335351 * 10 * distance - 1.23966075 * powf(10, 2));
    } else
        return 0;
}


Vector localizeWithOffensiveGoal() {
    const Vector realGoalToCenter = {0, 113.5};
    const Vector yellow_goalactualposition = {
        sensorValues.yellowgoal_relativeposition.angle -
            sensorValues.relativeBearing,
        sensorValues.yellowgoal_relativeposition.distance};
    const auto fakeCenter = - yellow_goalactualposition + realGoalToCenter;
    Vector actualCenter = {clipAngleto180degrees(fakeCenter.angle),
                           fakeCenter.distance};
    return actualCenter;
};
Vector localizeWithDefensiveGoal() {
    const Vector  realGoalToCenter = {180, 113.5};
    const Vector blue_goalactualposition = {
        sensorValues.bluegoal_relativeposition.angle -
            sensorValues.relativeBearing,
        sensorValues.bluegoal_relativeposition.distance};
    const auto fakeCenter = - blue_goalactualposition + realGoalToCenter;
    Vector actualCenter = {clipAngleto180degrees(fakeCenter.angle),
                           fakeCenter.distance};
    return actualCenter;
};

Vector localizeWithBothGoals() {
    return (localizeWithDefensiveGoal() + localizeWithOffensiveGoal()) / 2;
};

void processLidars() {
    double x_leftrelativetofield = 0;
    double x_rightrelativetofield = 0;
    double y_frontrelativetofield = 0;
    double y_backrelativetofield = 0;
    Point CameraPosition;
    
    for (int i = 0; i < 4; i++) {
        processedValues.lidarDistance[i] =
            constrain(sensorValues.lidardist[i] + LIDARSMEASUREMENTOFFSET,0,1000);
    }

    if (sensorValues.relativeBearing <= 45 && sensorValues.relativeBearing > -45) {
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
    } else if (sensorValues.relativeBearing > 45 && sensorValues.relativeBearing <= 135) {
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
    } else if (sensorValues.relativeBearing > 135 ||
               sensorValues.relativeBearing <= -135) {
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
    } else if (sensorValues.relativeBearing <= -45 &&
               sensorValues.relativeBearing > -135) {
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