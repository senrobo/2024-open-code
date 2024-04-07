#include "movement.h"
#include "config.h"
#include "main.h"

#ifdef ROBOT1
    #define FL_PWM_PIN 11
    #define FL_IN1_PIN 10
    #define FL_IN2_PIN 12
    #define BL_PWM_PIN 8
    #define BL_IN1_PIN 7
    #define BL_IN2_PIN 9
    #define FR_PWM_PIN 2
    #define FR_IN1_PIN 1
    #define FR_IN2_PIN 3
    #define BR_PWM_PIN 5
    #define BR_IN1_PIN 4
    #define BR_IN2_PIN 6

    #define FL_MULTIPLIER 1
    #define FR_MULTIPLIER 1
    #define BL_MULTIPLIER 1
    #define BR_MULTIPLIER 1

#endif

#ifdef ROBOT2
    #define FL_PWM_PIN 11
    #define FL_IN1_PIN 10
    #define FL_IN2_PIN 12
    #define BL_PWM_PIN 8
    #define BL_IN1_PIN 7
    #define BL_IN2_PIN 9
    #define FR_PWM_PIN 2
    #define FR_IN1_PIN 1
    #define FR_IN2_PIN 3
    #define BR_PWM_PIN 5
    #define BR_IN1_PIN 4
    #define BR_IN2_PIN 6

    #define FL_MULTIPLIER 1
    #define FR_MULTIPLIER 1
    #define BL_MULTIPLIER 1
    #define BR_MULTIPLIER 1
#endif

#define SIN34 0.55919290F
#define SIN56 0.82903757F
#define COS34 0.82903757F
#define COS56 0.55919290F

Movement::Movement(){};

void Movement::initialize() {
    // initalize pins
    pinMode(FL_PWM_PIN, OUTPUT);
    pinMode(FL_IN1_PIN, OUTPUT);
    pinMode(FL_IN2_PIN, OUTPUT);
    pinMode(BL_PWM_PIN, OUTPUT);
    pinMode(BL_IN1_PIN, OUTPUT);
    pinMode(BL_IN2_PIN, OUTPUT);
    pinMode(FR_PWM_PIN, OUTPUT);
    pinMode(FR_IN1_PIN, OUTPUT);
    pinMode(FR_IN2_PIN, OUTPUT);
    pinMode(BR_PWM_PIN, OUTPUT);
    pinMode(BR_IN1_PIN, OUTPUT);
    pinMode(BR_IN2_PIN, OUTPUT);
    // changing PWM resolution
    analogWriteResolution(10);

    analogWriteFrequency(FL_PWM_PIN, 146484);
    analogWriteFrequency(BL_PWM_PIN, 146484);
    analogWriteFrequency(FR_PWM_PIN, 146484);
    analogWriteFrequency(BR_PWM_PIN, 146484);
};

void Movement::updateParameters(double actualbearing, double actualdirection,
                                double actualvelocity) {
    _actualbearing = actualbearing;
    _actualdirection = actualdirection;
};

void Movement::setconstantDirection(Direction::constant params) {
    _targetdirection = params.value;
    _accelerate = false;
};

void Movement::setmovetoPointDirection(Direction::movetoPoint params) {
    _targetdirection = clipAngleto180degrees(
        (Vector::fromPoint(params.destination) - params.robotCoordinate).angle -
        params.robotBearing);

    _accelerate = false;
};

void Movement::setlinetrackDirection(Direction::linetrack params) {
    linetrackController.updateSetpoint(params.targetLineDepth);
    double correction = linetrackController.advance(params.lineDepth);
    // Serial.println(correction);
    if (params.trackLeftwards == true) {
        _targetdirection = correction * 90 + params.angleBisector - 90;
    } else {
        _targetdirection = (correction * -90) + params.angleBisector + 90;
    }

    _accelerate = false;
}

// In Movement.cpp
void Movement::setCurveTracking(Point robotPosition, double r, double h,
                                double k, bool track_left) {
    double distance_to_circle = abs(
        sqrt(pow(robotPosition.x - h, 2) + pow(robotPosition.y - k, 2)) - r);

    double theta_desired;
    if (track_left) {
        theta_desired =
            atan2(robotPosition.y - k, robotPosition.x - h) * 180 / PI + 90;
    } else {
        theta_desired =
            atan2(robotPosition.y - k, robotPosition.x - h) * 180 / PI - 90;
    }

    theta_desired =
        atan2(sin(theta_desired * PI / 180), cos(theta_desired * PI / 180)) *
        180 / PI;

    if (track_left == true) {
        _targetdirection = theta_desired;
    } else {
        _targetdirection = theta_desired;
    }
    Serial.println(theta_desired);
    // curveTrackingController.updateSetpoint(theta_desired);
    // double theta_robot = ;
    // double theta_error = theta_desired - theta_robot;
    // theta_error =
    //     atan2(sin(theta_error * PI / 180), cos(theta_error * PI / 180)) * 180
    //     / PI;
    // double control_signal = curveTrackingController.advance(theta_error);
    _targetdirection = theta_desired;
}

void Movement::setconstantVelocity(Velocity::constant params) {
    _targetvelocity = params.value;
};
void Movement::setstopatPointVelocity(Velocity::stopatPoint params) {
    auto correction = stopatPointController.advance(params.errordistance);

    _targetvelocity = abs(correction);
    _targetvelocity =
        constrain(_targetvelocity, params.minSpeed, params.maxSpeed);
};

void Movement::setconstantBearing(Bearing::constant params) {
    _targetbearing = params.targetValue;
    _actualbearing = params.actualBearing;
};
void Movement::setmoveBearingtoPoint(Bearing::moveBearingtoPoint params) {
    if (params.destination != _finalDestination) {
        _finalbearing = params.finalBearing;
        _initialbearing = _actualbearing;
        _initialrobotcoordinate = params.robotCoordinate;
    }
    _finalDestination = params.destination;
    double progress =
        (params.robotCoordinate - _initialrobotcoordinate).distance /
        (Vector::fromPoint(params.destination) - _initialrobotcoordinate)
            .distance;

    _targetbearing =
        _initialbearing + progress * (_finalbearing - _initialbearing);
};
void Movement::setBearingSettings(double minV, double maxV, double KP,
                                  double KD, double KI) {
    bearingController.updateLimits(minV, maxV, infinity());
    bearingController.updateGains(KP, KD, KI, 0.2);
};

void Movement::setAcceleration(bool accelerate, double accelerationMultiplier) {
    _accelerate = accelerate;
    _accelerationMultiplier = accelerationMultiplier;
}

void Movement::drive(Point robotPosition, double bearing, int dt_micros) {
    bearingController.updateSetpoint(_targetbearing);

    if (_targetbearing <= 90 && _targetbearing >= -90) {
        _movingbearing =
            bearingController.advance(clipAngleto180degrees(_actualbearing));
    } else if (_targetbearing > 90 && _targetbearing <= 135) {
        _movingbearing =
            bearingController.advance(clipAngleto180degrees(_actualbearing));
    } else if (_targetbearing < -90 && _targetbearing >= -135) {
        _movingbearing =
            bearingController.advance(clipAngleto180degrees(_actualbearing));
    } else if (_targetbearing > 135) {
        _movingbearing =
            bearingController.advance(clipAngleto360degrees(_actualbearing));
    } else {
        _movingbearing =
            bearingController.advance(-clipAngleto360degrees(_actualbearing));
    }

    double x = sind(_targetdirection);
    double y = cosd(_targetdirection);

    double bearingRadians = bearing * M_PI / 180.0;
    double rotationMatrix[2][2] = {{cosd(bearing), -sind(bearing)},
                                   {sind(bearing), cosd(bearing)}};

    double rotatedX = rotationMatrix[0][0] * x + rotationMatrix[0][1] * y;
    double rotatedY = rotationMatrix[1][0] * x + rotationMatrix[1][1] * y;

    // https://www.desmos.com/calculator/3hicyamp5q
    double slowdownSpeedMultipier =
        constrain(91 * powf(_targetvelocity, -1.0) * 0.27, 0, 1);

    _Xactualvelocity = _targetvelocity;
    _Yactualvelocity = _targetvelocity;



        if (robotPosition.x > X_AXIS_SLOWDOWN_START) {

            double deccel =
                constrain(X_AXIS_SLOWDOWN_SPEED * slowdownSpeedMultipier -
                              ((robotPosition.x - X_AXIS_SLOWDOWN_START) /
                               (X_AXIS_SLOWDOWN_END - X_AXIS_SLOWDOWN_START) *
                               X_AXIS_SLOWDOWN_SPEED * slowdownSpeedMultipier),
                          0, 1000);
            rotatedX = constrain(rotatedX, -700.0, deccel);

            if (rotatedX > 0) {
                _Xactualvelocity =
                    constrain(_Xactualvelocity, 0, X_AXIS_MAX_VELOCITY);
            }

        } else if (robotPosition.x < X_NEGATIVE_AXIS_SLOWDOWN_START) {
            double deccel = constrain(
                X_AXIS_SLOWDOWN_SPEED * slowdownSpeedMultipier -
                    ((-robotPosition.x + X_NEGATIVE_AXIS_SLOWDOWN_START) /
                     (X_NEGATIVE_AXIS_SLOWDOWN_END -
                      X_NEGATIVE_AXIS_SLOWDOWN_START) *
                     X_AXIS_SLOWDOWN_SPEED * slowdownSpeedMultipier),
                -1000, 0);

            if (rotatedX < 0) {
                _Xactualvelocity =
                    constrain(_Xactualvelocity, 0, X_AXIS_MAX_VELOCITY);
            }

            rotatedX = constrain(rotatedX, deccel, 600);
        }
        if (robotPosition.x < X_GOAL_WIDTH && robotPosition.x > -X_GOAL_WIDTH) {
            if (robotPosition.y > Y_AXIS_SLOWDOWN_START_GOAL) {
                double deccel = constrain(
                    Y_AXIS_SLOWDOWN_SPEED_GOAL * slowdownSpeedMultipier -
                        ((robotPosition.y - Y_AXIS_SLOWDOWN_START_GOAL) /
                         (Y_AXIS_SLOWDOWN_END_GOAL -
                          Y_AXIS_SLOWDOWN_START_GOAL) *
                         Y_AXIS_SLOWDOWN_SPEED_GOAL * slowdownSpeedMultipier),
                    0, 1000);
                rotatedY = constrain(rotatedY, -600.0, deccel);

                if (rotatedY > 0) {
                    _Yactualvelocity = constrain(_Yactualvelocity, 0,
                                                 Y_AXIS_MAX_VELOCITY_GOAL);
                }

            } else if (robotPosition.y < Y_NEGATIVE_AXIS_SLOWDOWN_START_GOAL) {
                double deccel = constrain(
                    Y_AXIS_SLOWDOWN_SPEED_GOAL * slowdownSpeedMultipier -
                        ((-robotPosition.y +
                          Y_NEGATIVE_AXIS_SLOWDOWN_START_GOAL) /
                         (Y_NEGATIVE_AXIS_SLOWDOWN_END_GOAL -
                          Y_NEGATIVE_AXIS_SLOWDOWN_START_GOAL) *
                         Y_AXIS_SLOWDOWN_SPEED_GOAL * slowdownSpeedMultipier),
                    -1000, 0);
                rotatedY = constrain(rotatedY, deccel, 600);

                if (rotatedY < 0) {
                    _Yactualvelocity = constrain(_Yactualvelocity, 0,
                                                 Y_AXIS_MAX_VELOCITY_GOAL);
                }
            }
        

        double adjustedX =
            rotationMatrix[0][0] * rotatedX + rotationMatrix[1][0] * rotatedY;
        double adjustedY =
            rotationMatrix[0][1] * rotatedX + rotationMatrix[1][1] * rotatedY;

        x = adjustedX;
        y = adjustedY;
    }

    const auto transformspeed = [this](double X_velocityDirection,
                                       double Y_velocityDirection,
                                       double angularComponent) {
        return (int)(_Xactualvelocity * X_velocityDirection +
                     _Yactualvelocity * Y_velocityDirection + angularComponent);
    };

    double angularComponent = _movingbearing * 1.3;

    double FLSp =
        transformspeed(x * SIN34, y * COS56, angularComponent) * FL_MULTIPLIER;
    double FRSp = transformspeed(x * -SIN34, y * COS56, -angularComponent) *
                  FR_MULTIPLIER;
    double BRSp =
        transformspeed(x * SIN34, y * COS56, -angularComponent) * BR_MULTIPLIER;
    double BLSp =
        transformspeed(x * -SIN34, y * COS56, angularComponent) * BL_MULTIPLIER;

    _accelerate == false;
    if (_accelerate == true) {
        if (FLSpeed > FLSp + ACCELERATION_ERROR) {
            FLSpeed -= dt_micros * _accelerationMultiplier;
        } else if (FLSpeed < FLSp - ACCELERATION_ERROR) {
            FLSpeed += dt_micros * _accelerationMultiplier;
        } else {
            FLSpeed = FLSp;
        }
        if (FRSpeed > FRSp + ACCELERATION_ERROR) {
            FRSpeed -= dt_micros * _accelerationMultiplier;
        } else if (FRSpeed < FRSp - ACCELERATION_ERROR) {
            FRSpeed += dt_micros * _accelerationMultiplier;
        } else {
            FRSpeed = FRSp;
        }
        if (BLSpeed > BLSp + ACCELERATION_ERROR) {
            BLSpeed -= dt_micros * _accelerationMultiplier;
        } else if (BLSpeed < BLSp - ACCELERATION_ERROR) {
            BLSpeed += dt_micros * _accelerationMultiplier;
        } else {
            BLSpeed = BLSp;
        }
        if (BRSpeed > BRSp + ACCELERATION_ERROR) {
            BRSpeed -= dt_micros * _accelerationMultiplier;
        } else if (BRSpeed < BRSp - ACCELERATION_ERROR) {
            BRSpeed += dt_micros * _accelerationMultiplier;
        } else {
            BRSpeed = BRSp;
        }
    } else {
        FLSpeed = FLSp;
        BRSpeed = BRSp;
        BLSpeed = BLSp;
        FRSpeed = FRSp;
    }

    // if (FLSpeed < 50 && FLSpeed > -50) { FLSpeed = 0; }
    // if (FRSpeed < 50 && FRSpeed > -50) { FRSpeed = 0; }
    // if (BLSpeed < 50 && BLSpeed > -50) { BLSpeed = 0; }
    // if (BRSpeed < 50 && BRSpeed > -50) { BRSpeed = 0; }

#ifdef ROBOT1
    digitalWriteFast(FL_IN1_PIN, FLSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(FL_IN2_PIN, FLSpeed > 0 ? HIGH : LOW);

    digitalWriteFast(FR_IN1_PIN, FRSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(FR_IN2_PIN, FRSpeed > 0 ? HIGH : LOW);

    digitalWriteFast(BR_IN1_PIN, BRSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(BR_IN2_PIN, BRSpeed > 0 ? HIGH : LOW);

    digitalWriteFast(BL_IN1_PIN, BLSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(BL_IN2_PIN, BLSpeed > 0 ? HIGH : LOW);

    analogWrite(FL_PWM_PIN, constrain(abs(FLSpeed), -800, 800));
    analogWrite(FR_PWM_PIN, constrain(abs(FRSpeed), -800, 800));
    analogWrite(BL_PWM_PIN, constrain(abs(BLSpeed), -800, 800));
    analogWrite(BR_PWM_PIN, constrain(abs(BRSpeed), -800, 800));
#endif
#ifdef ROBOT2
    digitalWriteFast(FL_IN1_PIN, FLSpeed > 0 ? HIGH : LOW);
    digitalWriteFast(FL_IN2_PIN, FLSpeed > 0 ? LOW : HIGH);

    digitalWriteFast(FR_IN1_PIN, FRSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(FR_IN2_PIN, FRSpeed > 0 ? HIGH : LOW);

    digitalWriteFast(BR_IN1_PIN, BRSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(BR_IN2_PIN, BRSpeed > 0 ? HIGH : LOW);

    digitalWriteFast(BL_IN1_PIN, BLSpeed > 0 ? HIGH : LOW);
    digitalWriteFast(BL_IN2_PIN, BLSpeed > 0 ? LOW : HIGH);

    analogWrite(FL_PWM_PIN, constrain(abs(FLSpeed), -700, 700));
    analogWrite(FR_PWM_PIN, constrain(abs(FRSpeed), -700, 700));
    analogWrite(BL_PWM_PIN, constrain(abs(BLSpeed), -700, 700));
    analogWrite(BR_PWM_PIN, constrain(abs(BRSpeed), -700, 700));
#endif
#ifdef DEBUG_MOVEMENT
    const auto printSerial = [](double value) {
        Serial.printf("%5d", (int)value);
    };

    Serial.print("FLSpeed: ");
    printSerial(FLSpeed);
    Serial.print(" | FRSpeed: ");
    printSerial(FRSpeed);
    Serial.print(" | BLSpeed: ");
    printSerial(BLSpeed);
    Serial.print(" | BRSpeed: ");
    printSerial(BRSpeed);
    Serial.print(" | TargetDirection: ");
    printSerial(_targetdirection);
    Serial.print(" | TargetVelocity: ");
    printSerial(_targetvelocity);
    Serial.print(" | XComponent: ");
    printDouble(Serial, x, 3, 3);
    Serial.print(" | YComponent: ");
    printDouble(Serial, y, 3, 3);
    Serial.print(" | YPosition: ");
    printSerial(robotPosition.y);
    Serial.print(" | Slowdown");
    printSerial(
        constrain(Y_AXIS_SLOWDOWN_SPEED_GOAL -
                      ((robotPosition.y - Y_AXIS_SLOWDOWN_START_GOAL) /
                       (Y_AXIS_SLOWDOWN_END_GOAL - Y_AXIS_SLOWDOWN_START_GOAL) *
                       Y_AXIS_SLOWDOWN_SPEED_GOAL),
                  0, 1000));

    Serial.println(" ");

#endif
};

double Movement::applySigmoid(double startSpeed, double endSpeed,
                              double progress, double constant) {
    const auto multiplier = 1 / (1 + powf(1200, constant * 2 * progress - 1));
    return startSpeed + (endSpeed - startSpeed) * multiplier;
};

std::vector<double> Movement::getmotorValues() {
    const auto x = sind(_targetdirection);
    const auto y = cosd(_targetdirection);

    const auto transformspeed = [this](double velocityDirection,
                                       double angularComponent) {
        return (int)(_targetvelocity * velocityDirection + angularComponent);
    };

    double angularComponent = _movingbearing * 1.3;

    double FLSpeed = transformspeed(x * SIN34 + y * COS56, angularComponent);
    double FRSpeed = transformspeed(x * -SIN34 + y * COS56, -angularComponent);
    double BRSpeed = transformspeed(x * SIN34 + y * COS56, -angularComponent);
    double BLSpeed = transformspeed(x * -SIN34 + y * COS56, angularComponent);

    return {FLSpeed, FRSpeed, BLSpeed, BRSpeed};
};
