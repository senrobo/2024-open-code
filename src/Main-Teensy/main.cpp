#include "main.h"
#include "SPI.h"
#include <Arduino.h>
#include <Servo.h>
#include <array>

#include "PacketSerial.h"
#include "config.h"
#include "movement.h"
#include "shared.h"

// hello

// put function declarations here:

Point KICK_POINT{-50, 50};

PacketSerial L3TeensySerial;
PacketSerial L1Serial;
PacketSerial L2toL1Serial;
Movement movement;

SensorValues sensorValues;
Solenoid solenoid;
TruthValues truthValues;
Servo frontDrib;
Execution execution;
LightArray lightArray;
ProcessedValues processedValues;

// https://www.desmos.com/calculator/5uexflvu3o

void receiveL3TxData(const byte *buf, size_t size) {
    // load payload
    L3Txpayloaf payload;
    // if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));
    processedValues.relativeBearing = payload.sensorValues.relativeBearing;
    processedValues.bluegoal_relativeposition =
        payload.sensorValues.bluegoal_relativeposition;
    processedValues.ball_relativeposition =
        payload.sensorValues.ball_relativeposition;
    processedValues.yellowgoal_relativeposition =
        payload.sensorValues.yellowgoal_relativeposition;
    processedValues.ball_relativeposition.angle += 0;
    processedValues.robot_position = payload.sensorValues.robot_position;
    processedValues.yellowgoal_exists = payload.sensorValues.yellowgoal_exists;
    processedValues.bluegoal_exists = payload.sensorValues.bluegoal_exists;
    processedValues.ballExists = payload.sensorValues.ballExists;
    execution.attackMode = payload.sensorValues.attackMode;

    for (int i = 0; i < 4; i++) {
        processedValues.lidarDistance[i] =
            payload.sensorValues.lidarDistance[i];
        processedValues.lidarConfidence[i] =
            payload.sensorValues.lidarConfidence[i];
    }

    return;
}

void driveBrushless(double value) { analogWrite(DRIBBLER_PWM_PIN, value); }

void movetoPoint(Point destination, double startSpeed = 400,
                 double endSpeed = 250, double multiplier = 0.9) {
    Vector localisation = Vector::fromPoint(processedValues.robot_position);
    double distance = (localisation - Vector::fromPoint(destination)).distance;
    movement.setconstantVelocity(Velocity::constant{movement.applySigmoid(
        startSpeed, endSpeed, (distance) / 20, multiplier)});
    movement.setmovetoPointDirection(Direction::movetoPoint{
        localisation, destination, processedValues.relativeBearing});
}

double averageballinCatchment;
double averageballinCatchmentTime = 0;
double averageballinCatchmentCount = 0;
double previous_averageballinCatchment = 0;

double avg_ballinCatchment(int dt, int delay) {
    averageballinCatchmentTime += dt;
    // Serial.println(sensorValues.ballinCatchment);
    if (averageballinCatchmentTime < delay) {
        averageballinCatchment += sensorValues.ballinCatchment;
        averageballinCatchmentCount += 1;
        return previous_averageballinCatchment;
    } else {
        averageballinCatchmentTime = 0;
        previous_averageballinCatchment =
            averageballinCatchment / averageballinCatchmentCount;
        averageballinCatchmentCount = 0;
        averageballinCatchment = 0;
        return previous_averageballinCatchment;
    }
}

double combinedBallDetected;
double averageBallDetectedTime = 0;
double averageBallDetectedCount = 0;
double averageBallDetected = 0;

double avg_ballDetected(int dt, int delay) {
    averageBallDetectedTime += dt;
    // Serial.println(sensorValues.ballDetected);
    if (averageBallDetectedTime < delay) {
        combinedBallDetected += processedValues.ballExists;
        averageBallDetectedCount += 1;
        return averageBallDetected;
    } else {
        averageBallDetectedTime = 0;
        averageBallDetected =
            averageBallDetectedCount > 0
                ? combinedBallDetected / averageBallDetectedCount
                : 0;
        averageBallDetectedCount = 0;
        combinedBallDetected = 0;
        return averageBallDetected;
    }
}

void setup() {

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(MuxInput1, INPUT);
    pinMode(MuxInput2, INPUT);
    pinMode(MuxInput3, INPUT);
    pinMode(DRIBBLER_PWM_PIN, OUTPUT);
    pinMode(Solenoid_Pin, OUTPUT);
    digitalWrite(Solenoid_Pin, LOW);

    delay(10);
    analogWriteResolution(10);
    Serial5.begin(500000);

    Serial.begin(9600);
    L3TeensySerial.setStream(
        &Serial5); // set serial stream to packet communcation to default serial

    L3TeensySerial.setPacketHandler(&receiveL3TxData);

    movement.initialize();
    attachBrushless();
}
long last = 0;

long dt_micros = 0;
long now = 0;
int kicktimeinMillis = 0;
int locationtime = 0;
int accelTrue = 0;
int direction = 0;
int ldrCounter = 0;

double frontVariance = 2;
double backVariance = 2;
double leftVariance = 2;
double rightVariance = 2;

double last_bearing = 0;

void loop() {
    dt_micros = loopTimeinmicros();

    // SETUP PHASE
    L3TeensySerial.update();
    L3TeensySerial.update();

    Vector actualball_position = {
        processedValues.ball_relativeposition.angle +
            processedValues.relativeBearing,
        processedValues.ball_relativeposition.distance};
    Point ballposition = {(actualball_position +
                           Vector::fromPoint(processedValues.robot_position))
                              .x(),
                          (actualball_position +
                           Vector::fromPoint(processedValues.robot_position))
                              .y()};

    findLine(dt_micros);
    processedValues.averageCatchmentValues =
        avg_ballinCatchment(dt_micros, 100000);

    processedValues.averageballExists = avg_ballDetected(dt_micros, 100000);
    movement.setAcceleration(false, 0.0005);

// Testing Defence Robot
#ifdef TEST_DEFENCE_ROBOT
    execution.attackMode = 0;
#endif


    if (solenoid.kick == 1024 ||
        millis() - execution.lastkickTime < MIN_KICK_TIME) {
        execution.dribblerSpeed = 150;
        if ((solenoid.kick == 1024 ||
             millis() - execution.lastkickTime < MIN_KICK_TIME) &&
            millis() - execution.lastdribblerKickDelayTime >
                DRIBBLER_KICK_DELAY_TIME) {
            if (solenoid.kick == 1024) {

                execution.dribblerSpeed = 150;
                execution.lastkickTime = millis();
                digitalWrite(Solenoid_Pin, HIGH);

            } else {

                execution.dribblerSpeed = 150;
                digitalWrite(Solenoid_Pin, HIGH);
            }
            if (solenoid.kick == 1024) { execution.lastkickTime = millis(); }
        } else {
            digitalWrite(Solenoid_Pin, LOW);
        }

    } else {
        execution.lastdribblerKickDelayTime = millis();
        digitalWrite(Solenoid_Pin, LOW);
    }

    if (processedValues.ball_relativeposition.distance > 50 &&
        processedValues.averageCatchmentValues > CATCHMENT_THRESHOLD) {

#ifdef ROBOT1
        execution.dribblerSpeed = 120;
#endif
#ifdef ROBOT2
        execution.dribblerSpeed = 120;
#endif
    } else if (execution.strategy == 2) {
        execution.dribblerSpeed = 170;
    } else {
        execution.dribblerSpeed = 160;
    }

#ifdef DEBUG_THRESHOLD_VALUES

    for (int i = 0; i < 36; i++) {
        lightArray.LDRThresholds[i] =
            lightArray.minRecordedValue[i] +
            (lightArray.maxRecordedValue[i] - lightArray.minRecordedValue[i]) *
                0.1;
        if (i == 35) {
            Serial.print(lightArray.LDRThresholds[i]);
            Serial.println(" ");
        } else {
            Serial.print(lightArray.LDRThresholds[i]);
            Serial.print(" , ");
        }
    }
#endif
#ifdef LOCALISE_CODE
    Point location = {0, 0};
    truthValues.bearingtoField =
        clipAngleto180degrees(processedValues.relativeBearing);
    movement.setconstantBearing(Bearing::constant{LOCALISE_CODE_TARGET_BEARING,
                                                  truthValues.bearingtoField});
    movement.setAcceleration(true, 0.0006);
    locationtime += dt_micros;
    if (locationtime <= 5000000)
        location = {1000, 0};
    else if (locationtime <= 10000000)
        location = {33, 490}; // 33, 49
    else if (locationtime <= 15000000)
        location = {33, -490}; // 33,-49
    else if (locationtime <= 20000000)
        location = {-330, -49};
    else
        locationtime = 0;
    movetoPoint(location, 600, 250, 4);
    execution.targetBearing = 0;
    movement.setconstantBearing(Bearing::constant{
        execution.targetBearing, processedValues.relativeBearing});
#endif
#ifdef NORMAL_CODE

    if (execution.attackMode == 0) {

        movement.setBearingSettings(
            MIN_BEARING_CORRECTION, MAX_BEARING_CORRECTION,
            DEFENCE_ROBOT_BEARING_KP, DEFENCE_ROBOT_BEARING_KD,
            DEFENCE_ROBOT_BEARING_KI);

        if (sensorValues.onLine == 2) {
            if (clipAngleto180degrees(sensorValues.angleBisector) <= 90 &&
                clipAngleto180degrees(sensorValues.angleBisector) >= -90) {
                processedValues.defenceRobotAngleBisector =
                    clipAngleto180degrees(180 + sensorValues.angleBisector);
                processedValues.defenceRobotDepthinLine =
                    (1.0 - sensorValues.depthinLine) + 1.0;
            } else {
                processedValues.defenceRobotAngleBisector =
                    sensorValues.angleBisector;
                processedValues.defenceRobotDepthinLine =
                    sensorValues.depthinLine;
            }
        }
        // if robot is no online
        if (sensorValues.onLine == 1) {

            movement.setconstantBearing(
                Bearing::constant{0, processedValues.relativeBearing});
            movement.setconstantVelocity(Velocity::constant{400});
            movement.setconstantDirection(Direction::constant{
                processedValues.bluegoal_relativeposition.angle});

        }

        // if robot is online
        else if (sensorValues.onLine == 2) {
            if (processedValues.lidarDistance[1] < 50 || processedValues.lidarDistance[3] < 50) {
                if (processedValues.lidarDistance[3] < 50) {
                    movement.setlinetrackDirection(Direction::linetrack{
                        processedValues.defenceRobotDepthinLine,
                        processedValues.defenceRobotAngleBisector,
                        DEFENCE_DEPTH_IN_LINE, true});

                    movement.setconstantVelocity(
                        Velocity::constant{DEFENCE_REJECTION_VELOCITY});

                } else {
                    movement.setlinetrackDirection(Direction::linetrack{
                        processedValues.defenceRobotDepthinLine,
                        processedValues.defenceRobotAngleBisector,
                        DEFENCE_DEPTH_IN_LINE, false});

                    movement.setconstantVelocity(
                        Velocity::constant{DEFENCE_REJECTION_VELOCITY});
                }
                } else if (averageBallDetected <= 0.1) {
                    if (processedValues.bluegoal_relativeposition.angle > 0) {
                        movement.setlinetrackDirection(Direction::linetrack{
                            processedValues.defenceRobotDepthinLine,
                            processedValues.defenceRobotAngleBisector,
                            DEFENCE_DEPTH_IN_LINE, true});

                        double progress =
                            processedValues.bluegoal_relativeposition.angle /
                            90;

                        movement.setconstantVelocity(
                            Velocity::constant{DEFENCE_NO_BALL_VELOCITY});

                    } else {
                        movement.setlinetrackDirection(Direction::linetrack{
                            processedValues.defenceRobotDepthinLine,
                            processedValues.defenceRobotAngleBisector,
                            DEFENCE_DEPTH_IN_LINE, false});
                        double progress =
                            processedValues.bluegoal_relativeposition.angle /
                            90;
                        movement.setconstantVelocity(
                            Velocity::constant{DEFENCE_NO_BALL_VELOCITY});
                    }
                } else {
                    if (clipAngleto360degrees(
                            processedValues.bluegoal_relativeposition.angle) >
                            clipAngleto360degrees(
                                processedValues.ball_relativeposition.angle -
                                180) &&
                        processedValues.lidarDistance[3] >=
                            DEFENCE_STOP_LINE_TRACK_LIDAR_DIST) {
                        movement.setlinetrackDirection(Direction::linetrack{
                            processedValues.defenceRobotDepthinLine,
                            processedValues.defenceRobotAngleBisector,
                            DEFENCE_DEPTH_IN_LINE, false});
                        double progress =
                            (processedValues.bluegoal_relativeposition.angle -
                             (processedValues.ball_relativeposition.angle -
                              180)) /
                            90;
                        movement.setconstantVelocity(
                            Velocity::constant{movement.applySigmoid(
                                DEFENCE_TRACKBALL_MAX_VELOCITY,
                                DEFENCE_TRACKBALL_MIN_VELOCITY, progress,
                                DEFENCE_ACCELERATION_MULTIPLIER)});
                        Serial.println("sad");
                    }

                    else if (clipAngleto360degrees(
                                 processedValues.bluegoal_relativeposition
                                     .angle) <
                                 clipAngleto360degrees(
                                     processedValues.ball_relativeposition
                                         .angle -
                                     180) &&
                             processedValues.lidarDistance[1] >=
                                 DEFENCE_STOP_LINE_TRACK_LIDAR_DIST) {

                        movement.setlinetrackDirection(Direction::linetrack{
                            processedValues.defenceRobotDepthinLine,
                            processedValues.defenceRobotAngleBisector,
                            DEFENCE_DEPTH_IN_LINE, true});

                        double progress =
                            (-clipAngleto360degrees(
                                 processedValues.bluegoal_relativeposition
                                     .angle) +
                             clipAngleto360degrees(
                                 processedValues.ball_relativeposition.angle -
                                 180)) /
                            90;
                        movement.setconstantVelocity(
                            Velocity::constant{movement.applySigmoid(
                                DEFENCE_TRACKBALL_MAX_VELOCITY,
                                DEFENCE_TRACKBALL_MIN_VELOCITY, progress,
                                DEFENCE_ACCELERATION_MULTIPLIER)});
                        Serial.println("sad1");
                    }
                }
            movement.setconstantBearing(
                Bearing::constant{0, processedValues.relativeBearing});
        }
        last_bearing = processedValues.relativeBearing;

        // movement.setconstantBearing(Bearing::constant{0.0,0});

    #ifdef DEBUG_DEFENCE_BOT
        const auto printSerial = [](double value) {
            Serial.printf("%5d", (int)value);
        };
        Serial.print("Actual Angle Bisector: ");
        printSerial(sensorValues.angleBisector);
        Serial.print(" | Actual DepthinLine: ");
        Serial.print(sensorValues.depthinLine);
        Serial.print("| Angle Bisector: ");
        printSerial(processedValues.defenceRobotAngleBisector);
        Serial.print(" | DepthinLine: ");
        Serial.print(processedValues.defenceRobotDepthinLine);
        Serial.print(" | FirstValue: ");
        Serial.print(sensorValues.linetrackldr1);
        Serial.print(" | SecondValue: ");
        Serial.print(sensorValues.linetrackldr1);
        Serial.println(" ");

    #endif
    }

    // Serial.println(sensorValues.ballinCatchment);

    // Serial.println(processedValues.ball_relativeposition.angle);

    #ifdef DEBUG_LIDAR
    Serial.print(sensorValues.lidardist[0]);
    Serial.print(", ");
    Serial.print(sensorValues.lidardist[1]);
    Serial.print(", ");
    Serial.print(sensorValues.lidardist[2]);
    Serial.print(", ");
    Serial.println(sensorValues.lidardist[3]);
    Serial.print(", ");
    Serial.println(sensorValues.relativeBearing);
    Serial.print(", ");
    Serial.println(sensorValues.ball_relativeposition.distance);
    Serial.print(", ");
    Serial.println(sensorValues.bluegoal_relativeposition.distance);
    Serial.print(", ");
    Serial.println(sensorValues.yellowgoal_relativeposition.distance);

    #endif

    else {

        movement.setBearingSettings(-500, 500, 3, 100, 0);
        // calculate time without ball
        if (processedValues.averageCatchmentValues > CATCHMENT_THRESHOLD) {
            execution.catchmentTime = millis();
        }

        if (execution.strategy == 2) {
            if (ballposition.x >= 0) {
                execution.targetBearing = 180;
                execution.strategy2Quadrant = 1;
                KICK_POINT = {50, 30};
            } else if (ballposition.x < 0) {
                execution.targetBearing = 180;
                execution.strategy2Quadrant = 4;
                KICK_POINT = {-50, 30};
            }
        } else if (execution.strategy == 1) {
            execution.targetBearing = 0;
        }

        if (sensorValues.onLine == 2) {
            movement.setconstantDirection(Direction::constant{
                clipAngleto180degrees(180 + sensorValues.angleBisector)});
            movement.setconstantVelocity(Velocity::constant{600});
        }

        else if (processedValues.averageCatchmentValues >=
                 CATCHMENT_THRESHOLD) { // 720

            solenoid.kick = 0;
            if ((((ballposition.x < -X_BALL_STRATEGY2_RANGE ||
                   ballposition.x > X_BALL_STRATEGY2_RANGE) &&
                  (ballposition.y < Y_BALL_STRATEGY2) &&
                  processedValues.robot_position.y < -40 &&
                  processedValues.averageballExists >= 0.98 &&
                  processedValues.robot_position.y > ballposition.y) ||
                 millis() - execution.lastTurnBackwardsTime <
                     MIN_TURN_AROUND_TIME)) {

                if ((ballposition.x < -X_BALL_STRATEGY2_RANGE ||
                     ballposition.x > X_BALL_STRATEGY2_RANGE) &&
                    (ballposition.y < Y_BALL_STRATEGY2) &&
                    averageBallDetected > 0.95) {
                    execution.lastTurnBackwardsTime = millis();
                }

                execution.strategy = 2;

            } else {
                execution.strategy = 1;
            }

            if (processedValues.ballExists == 0) {
                movetoPoint(DEFAULT_POSITION);
            }

            // else if (ballposition.x < X_AXIS_BALL_RANGE &&
            //          ballposition.x > -X_AXIS_BALL_RANGE &&
            //          ballposition.y > Y_AXIS_BALL_RANGE) {

            //     movetoPoint(DEFAULT_POSITION);
            // }

            else {

                // if ((processedValues.ball_relativeposition.angle + processedValues.relativeBearing < 60 && 
                // processedValues.ball_relativeposition.angle + processedValues.relativeBearing > 20 )||
                // (processedValues.ball_relativeposition.angle + processedValues.relativeBearing > -60 && 
                // processedValues.ball_relativeposition.angle + processedValues.relativeBearing < -20)){
                //      if (processedValues.ball_relativeposition.angle + processedValues.relativeBearing < 60 && 
                // processedValues.ball_relativeposition.angle + processedValues.relativeBearing > 20 ){
                //         movetoPoint({ballposition.x - 10, ballposition.y - 25}, 700,
                //                     300, 0.8);
                // }
                //     movetoPoint({ballposition.x + 10 , ballposition.y - 25}, 700, 300, 0.8);
                // }
                // else{
                    movement.setconstantVelocity(
                        Velocity::constant{movement.applySigmoid(
                            700, 300,
                            curveAroundBallMultiplier(
                                processedValues.ball_relativeposition.angle +
                                    processedValues.relativeBearing,
                                processedValues.ball_relativeposition.distance -
                                    20,
                                70 - 20),
                            1.1)});
                    solenoid.kick = 0;
                    movement.setconstantDirection(Direction::constant{
                        ballAngleOffset(
                            processedValues.ball_relativeposition.distance,
                            processedValues.ball_relativeposition.angle) +
                        processedValues.ball_relativeposition.angle});
                }
        } else if (processedValues.averageCatchmentValues <=
                   CATCHMENT_THRESHOLD) { // 720
            if (execution.strategy2GotBall == false){
                if (processedValues.robot_position.x >= 0) {
                    execution.targetBearing = 180;
                    execution.strategy2Quadrant = 1;
                    KICK_POINT = {50, 30};
                } else if (processedValues.robot_position.x < 0) {
                    execution.targetBearing = 180;
                    execution.strategy2Quadrant = 4;
                    KICK_POINT = {-50, 30};
                }
            }
            execution.strategy2GotBall = true;
        
            if (execution.strategy == 1) {
                movement.setBearingSettings(-80, 80, 3, 0, 0);

                execution.targetBearing =
                    processedValues.relativeBearing +
                    processedValues.yellowgoal_relativeposition.angle;

                if (processedValues.lidarDistance[0] < 40 &&
                        processedValues.lidarConfidence[0] == 0 ||
                    millis() - execution.lastavoidTime < 1000) {
                execution.dribblerSpeed = 170;
                    if (processedValues.lidarDistance[0] < 40 &&
                        processedValues.lidarConfidence[0] == 0) {

                        execution.lastavoidTime = millis();
                        execution.setAvoidBotDirection == true;
                        if (processedValues.robot_position.x <= 0) {
                            execution.direction = 90;
                        } else {
                            execution.direction = -90;
                        }
                    }

                    movement.setconstantDirection(
                        Direction::constant{execution.direction});
                    movement.setconstantVelocity(Velocity::constant{500});
                    movement.setAcceleration(true, 0.001);
                    

                } else if (processedValues.yellowgoal_relativeposition
                               .distance < 90) {

                    if ((processedValues.robot_position.x < 30 ||
                        processedValues.robot_position.x > -30) && processedValues.yellowgoal_relativeposition.distance < 60) {
                        movement.setconstantVelocity(Velocity::constant{500});
                        movement.setconstantDirection(Direction::constant{
                            processedValues.yellowgoal_relativeposition.angle});

                        solenoid.kick = 1024;
                    } 
                    else if ((processedValues.relativeBearing <
                                    execution.targetBearing +
                                        KICK_BEARING_ERROR && // NEED TO
                                                              // RECRAFT!!!
                                processedValues.relativeBearing >
                                    execution.targetBearing -
                                        KICK_BEARING_ERROR) &&
                               (millis() - execution.catchmentTime) >
                                   CATCH_BALL_DELAY_TIME) {
                        solenoid.kick = 1024;

                    } else if ((millis() - execution.catchmentTime) >
                               CATCH_BALL_DELAY_TIME){

                        movement.setconstantVelocity(Velocity::constant{500});
                        movement.setconstantDirection(Direction::constant{
                            processedValues.yellowgoal_relativeposition.angle});
                        execution.dribblerSpeed = 170;
                        movement.setAcceleration(true, 0.001);

                        //solenoid.kick = 1024;
                        }
                        else{
                            movement.setconstantVelocity(
                                Velocity::constant{600});
                            movement.setAcceleration(true, 0.004);
                        }
                } else {

                    movement.setconstantVelocity(Velocity::constant{600});
                    movement.setconstantDirection(Direction::constant{
                        processedValues.yellowgoal_relativeposition.angle});
                }
            }

            else if (execution.strategy == 2) {
                
                execution.dribblerSpeed = 165;
                if (processedValues.relativeBearing <
                        processedValues.yellowgoal_relativeposition.angle +
                            processedValues.relativeBearing + 10 &&
                    processedValues.relativeBearing >
                        processedValues.yellowgoal_relativeposition.angle +
                            processedValues.relativeBearing - 10 &&
                    processedValues.relativeBearing > -120 &&
                    processedValues.relativeBearing < 120) {
                    solenoid.kick = 1024;
                    movement.setBearingSettings(0, 1, 0.00001, 0, 0);
                    execution.targetBearing =
                        processedValues.yellowgoal_relativeposition.angle +
                        processedValues.relativeBearing;
                    movement.setconstantVelocity(Velocity::constant{0});
                    execution.dribblerSpeed = 130;
                }

                else if ((processedValues.robot_position.x - KICK_POINT.x <
                              X_LOCALISATION_ERROR_THRESHOLD &&
                          processedValues.robot_position.x - KICK_POINT.x >
                              -X_LOCALISATION_ERROR_THRESHOLD &&
                          processedValues.robot_position.y - KICK_POINT.y <
                              Y_LOCALISATION_ERROR_THRESHOLD &&
                          processedValues.robot_position.y - KICK_POINT.y >
                              -Y_LOCALISATION_ERROR_THRESHOLD &&
                          (millis() - execution.catchmentTime) >
                              CATCH_BALL_DELAY_TIME) ||
                         millis() - execution.lastTurnandKickTime < 3000) {
                    if (processedValues.robot_position.x - KICK_POINT.x <
                            X_LOCALISATION_ERROR_THRESHOLD &&
                        processedValues.robot_position.x - KICK_POINT.x >
                            -X_LOCALISATION_ERROR_THRESHOLD &&
                        processedValues.robot_position.y - KICK_POINT.y <
                            Y_LOCALISATION_ERROR_THRESHOLD &&
                        processedValues.robot_position.y - KICK_POINT.y >
                            -Y_LOCALISATION_ERROR_THRESHOLD &&
                        (millis() - execution.catchmentTime) >
                            CATCH_BALL_DELAY_TIME) {
                        execution.lastTurnandKickTime = millis();
                    }

    #ifdef TURN_AND_KICK_STRATEGY
                    execution.targetBearing =
                        (processedValues.relativeBearing +
                         processedValues.yellowgoal_relativeposition.angle);
                    movement.setBearingSettings(94, 95, 4, 0, 0);
                    movement.setconstantVelocity(Velocity::constant{0});
    #endif
    #ifdef FLICK_STRATEGY
                    if (execution.strategy2Quadrant == 1) {
                        if (processedValues.relativeBearing > 0) {
                            movement.setBearingSettings(300, 301, 4, 0, 0);
                        } else {
                            movement.setBearingSettings(-300, 300, 4, 0, 0);
                            execution.targetBearing = -90;
                        }
                    } else if (execution.strategy2Quadrant == 4) {
                        if (processedValues.relativeBearing <= 0) {
                            movement.setBearingSettings(-301, -300, 4, 0, 0);
                        } else {
                            movement.setBearingSettings(-300, 300, 4, 0, 0);
                            execution.targetBearing = 90;
                        }
                    }

    #endif

                    // movetoPoint(KICK_POINT);
                }

                else if (millis() - execution.catchmentTime >
                         CATCH_BALL_DELAY_TIME) {
                    Vector localisation = {processedValues.robot_position.x,
                                           processedValues.robot_position.y};
                    movetoPoint(KICK_POINT);
                    movement.setAcceleration(true, 0.0003);
                } else {
                    movement.setconstantVelocity(Velocity::constant{0});
                }
            }
        }
        movement.setconstantBearing(Bearing::constant{
            execution.targetBearing, processedValues.relativeBearing});
    }

#endif
#ifdef DEBUG_EVERYTHING

    // lightRing
    Serial.print("onLine: ");
    printDouble(Serial, sensorValues.onLine, 2, 0);
    Serial.print(" | angleBisector: ");
    printDouble(Serial, sensorValues.angleBisector, 3, 0);
    Serial.print(" | depthinLine: ");
    printDouble(Serial, sensorValues.depthinLine, 1, 3);
    Serial.print(" | firstLDR: ");
    printDouble(Serial, sensorValues.linetrackldr1, 3, 0);
    Serial.print(" | secondLDR: ");
    printDouble(Serial, sensorValues.linetrackldr2, 3, 0);
    Serial.print(" | catchment: ");
    printDouble(Serial, sensorValues.ballinCatchment, 3, 0);
    Serial.print(" | catchmentAverage: ");
    printDouble(Serial, processedValues.averageCatchmentValues, 3, 0);
    // L3 Data
    Serial.print(" | bearing: ");
    printDouble(Serial, processedValues.relativeBearing, 3, 1);
    Serial.print(" | frontLidar: ");
    printDouble(Serial, processedValues.lidarDistance[0], 3, 0);
    Serial.print(" | rightLidar: ");
    printDouble(Serial, processedValues.lidarDistance[1], 3, 0);
    Serial.print(" | backLidar: ");
    printDouble(Serial, processedValues.lidarDistance[2], 3, 0);
    Serial.print(" | leftLidar: ");
    printDouble(Serial, processedValues.lidarDistance[3], 3, 0);

    Serial.print(" | frontLidarConf: ");
    printDouble(Serial, processedValues.lidarConfidence[0], 3, 0);
    Serial.print(" | rightLidarConf: ");
    printDouble(Serial, processedValues.lidarConfidence[1], 3, 0);
    Serial.print(" | backLidarConf: ");
    printDouble(Serial, processedValues.lidarConfidence[2], 3, 0);
    Serial.print(" | leftLidarConf: ");
    printDouble(Serial, processedValues.lidarConfidence[3], 3, 0);
    Serial.print(" | attackGoalAngle: ");
    printDouble(Serial, processedValues.yellowgoal_relativeposition.angle, 3,
                1);
    Serial.print(" | attackGoalDist: ");
    printDouble(Serial, processedValues.yellowgoal_relativeposition.distance, 3,
                1);

    Serial.print(" | defenceGoalAngle: ");
    printDouble(Serial, processedValues.bluegoal_relativeposition.angle, 3, 1);
    Serial.print(" | defenceGoalDist: ");
    printDouble(Serial, processedValues.bluegoal_relativeposition.distance, 3,
                1);
    Serial.print(" | ballAngle: ");
    printDouble(Serial, processedValues.ball_relativeposition.angle, 3, 1);
    Serial.print(" | ballDist: ");
    printDouble(Serial, processedValues.ball_relativeposition.distance, 3, 1);
    // processed Values
    Serial.print(" | ballExistence: ");
    printDouble(Serial, processedValues.averageballExists, 1, 2);
    Serial.print(" | attackGoal Existence: ");
    printDouble(Serial, processedValues.yellowgoal_exists, 1, 1);
    Serial.print(" | defenceGoal Existence: ");
    printDouble(Serial, processedValues.bluegoal_exists, 1, 1);
    // Location
    Serial.print(" | X_position: ");
    printDouble(Serial, processedValues.robot_position.x, 3, 0);
    Serial.print(" | Y_position: ");
    printDouble(Serial, processedValues.robot_position.y, 3, 0);
    Serial.print(" | X_BALL_position: ");
    printDouble(Serial, ballposition.x, 3, 0);
    Serial.print(" | Y_BALL_position: ");
    printDouble(Serial, ballposition.y, 3, 0);
    Serial.print(" | targetBearing: ");
    printDouble(Serial, execution.targetBearing, 3, 0);
    Serial.print(" | currentMode: ");
    printDouble(Serial, execution.attackMode, 1, 1);
    Serial.print(" | dt: ");
    printDouble(Serial, dt_micros, 4, 1);
    Serial.print(" | dribblerSpeed: ");
    printDouble(Serial, execution.dribblerSpeedAccel, 4, 1);
    Serial.println("");
#endif

    movement.drive(
        {processedValues.robot_position.x, processedValues.robot_position.y},
        processedValues.relativeBearing, dt_micros);
#ifdef ROBOT1
    execution.dribblerSpeed += 60;
#endif
#ifdef ROBOT2
    execution.dribblerSpeed += 50;
#endif
    if (execution.dribblerSpeedAccel < execution.dribblerSpeed - 10) {
        execution.dribblerSpeedAccel += dt_micros * 0.00003;
    } else if (execution.dribblerSpeedAccel > execution.dribblerSpeed + 10) {
        execution.dribblerSpeedAccel -= dt_micros * 0.00003;
    } else {
        execution.dribblerSpeedAccel = execution.dribblerSpeed;
    }

    driveBrushless(execution.dribblerSpeedAccel);
}