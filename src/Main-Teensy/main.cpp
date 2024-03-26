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

Point KICK_POINT{0, 40};

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
    processedValues.ball_relativeposition.angle += 5;
    processedValues.robot_position = payload.sensorValues.robot_position;
    processedValues.yellowgoal_exists = payload.sensorValues.yellowgoal_exists;
    processedValues.bluegoal_exists = payload.sensorValues.bluegoal_exists;
    processedValues.ballExists = payload.sensorValues.ballExists;

    for (int i = 0; i < 4; i++) {
        processedValues.lidarDistance[i] = payload.sensorValues.lidarDistance[i];
        processedValues.lidarConfidence[i] = payload.sensorValues.lidarConfidence[i];
    }

    return;
}

void driveBrushless(double MINPWM, double MAXPWM, int dt = 100) {
    if (MAXPWM >= MINPWM) {
        for (int i = MINPWM; i <= MAXPWM; i = i + 1) {
            analogWrite(DRIBBLER_PWM_PIN, i);
            Serial.println(i);
            delay(dt);
        }
    } else {
        for (int i = MAXPWM; i <= MINPWM; i = i - 1) {
            analogWrite(DRIBBLER_PWM_PIN, i);
            Serial.println(i);
            delay(dt);
        }
    }
}



void movetoPoint(Point destination) {
    Vector localisation = Vector::fromPoint(processedValues.robot_position);
    double distance = (localisation - Vector::fromPoint(destination)).distance;
    movement.setconstantVelocity(Velocity::constant{
        movement.applySigmoid(400, 250, (distance) / 20, 0.9)});
    movement.setmovetoPointDirection(
        Direction::movetoPoint{localisation, destination});
}


int averageballinCatchment;
int averageballinCatchmentTime = 0;
int averageballinCatchmentCount = 0;
int previous_averageballinCatchment = 0;

int avg_ballinCatchment(int dt, int delay) {
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



void setup() {

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(MuxInput1, INPUT);
    pinMode(MuxInput2, INPUT);
    pinMode(MuxInput3, INPUT);
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
long dt = 0;
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
    dt = loopTimeinMillis();
    processedValues.averageCatchmentValues = avg_ballinCatchment(dt,10);

    // SETUP PHASE
    L3TeensySerial.update();
    L3TeensySerial.update();

    findLine();

    if (solenoid.kick == 1024) {
        digitalWrite(Solenoid_Pin, HIGH);
    } else {
        digitalWrite(Solenoid_Pin, LOW);
    }
    if (processedValues.ball_relativeposition.distance > 50){
        analogWrite(DRIBBLER_PWM_PIN, 180);
    }
    else{
        analogWrite(DRIBBLER_PWM_PIN, BRUSHLESS_DEFAULT_SPEED);
    }


#ifdef DEBUG_THRESHOLD_VALUES
    lightArray.highValues[sensorValues.linetrackldr2] =
        sensorValues.linetrackldr1;
    for (int i = 0; i < 36; i++) {
        lightArray.calculatedthesholdValue[i] =
            lightArray.minRecordedValue[i] +
            (lightArray.maxRecordedValue[i] - lightArray.minRecordedValue[i]) *
                0.3;
        if (i == 35) {
            Serial.print(lightArray.calculatedthesholdValue[i]);
            Serial.println(" ");
        } else {
            Serial.print(lightArray.calculatedthesholdValue[i]);
            Serial.print(" , ");
        }
    }
#endif

#ifdef DEFENCE_BOT_CODE
  
    if (sensorValues.onLine == 1){
        movement.setconstantDirection(Direction::constant{processedValues.bluegoal_relativeposition.angle});
        movement.setconstantVelocity(Velocity::constant{500});
        movement.setconstantBearing(Bearing::constant{last_bearing, processedValues.relativeBearing});
    }

    else if (sensorValues.onLine == 2 && processedValues.ballExists == 1) {
        if (clipAngleto180degrees(sensorValues.angleBisector) <= 90 &&
            clipAngleto180degrees(sensorValues.angleBisector) >= -90) {
            processedValues.defenceRobotAngleBisector =
                clipAngleto180degrees(180 + sensorValues.angleBisector);
            processedValues.defenceRobotDepthinLine =
                (1.0 - sensorValues.depthinLine) + 1.0;
        } else {
            processedValues.defenceRobotAngleBisector =
                sensorValues.angleBisector;
            processedValues.defenceRobotDepthinLine = sensorValues.depthinLine;
        }

        if (processedValues.ballExists == 0) {
            movement.setconstantVelocity(Velocity::constant{0});
            movement.setconstantDirection(Direction::constant{0});
        } else if (lightArray.RAWLDRVALUES[0] > lightArray.LDRThresholds[0]) {
            movement.setconstantVelocity(Velocity::constant{500});
            movement.setconstantDirection(Direction::constant{0});
        } else if (lightArray.RAWLDRVALUES[18] > lightArray.LDRThresholds[18]) {
            movement.setconstantVelocity(Velocity::constant{500});
            movement.setconstantDirection(Direction::constant{180});

        }
        else if (clipAngleto360degrees(
                       processedValues.bluegoal_relativeposition.angle) >

                       clipAngleto360degrees(
                           processedValues.ball_relativeposition.angle - 180) &&
                   processedValues.lidarDistance[3] >=
                       DEFENCE_STOP_LINE_TRACK_LIDAR_DIST) {
            movement.setlinetrackDirection(
                Direction::linetrack{processedValues.defenceRobotDepthinLine,
                                     processedValues.defenceRobotAngleBisector,
                                     DEFENCE_DEPTH_IN_LINE, false});
            double progress =
                (processedValues.bluegoal_relativeposition.angle -
                 (processedValues.ball_relativeposition.angle - 180)) /
                90;
            movement.setconstantVelocity(
                Velocity::constant{movement.applySigmoid(
                    600, 0, progress, DEFENCE_ACCELERATION_MULTIPLIER)});
            // movement.setconstantVelocity(Velocity::constant{400});
        }

        else if (clipAngleto360degrees(
                     processedValues.bluegoal_relativeposition.angle) <
                     clipAngleto360degrees(
                         processedValues.ball_relativeposition.angle - 180) &&
                 processedValues.lidarDistance[1] >=
                     DEFENCE_STOP_LINE_TRACK_LIDAR_DIST) {
                    
            movement.setlinetrackDirection(
                Direction::linetrack{processedValues.defenceRobotDepthinLine,
                                     processedValues.defenceRobotAngleBisector,
                                     DEFENCE_DEPTH_IN_LINE, true});

            double progress =
                (-clipAngleto360degrees(
                     processedValues.bluegoal_relativeposition.angle) +
                 clipAngleto360degrees(
                     processedValues.ball_relativeposition.angle - 180)) /
                90;
            movement.setconstantVelocity(
                Velocity::constant{movement.applySigmoid(
                    600, 0, progress, DEFENCE_ACCELERATION_MULTIPLIER)});

            // movement.setconstantVelocity(Velocity::constant{400});
            //Serial.print("kj");
        }

        // else if (sensorValues.lidardist[3] <
        //          DEFENCE_STOP_LINE_TRACK_LIDAR_DIST) {
        //     movement.setconstantVelocity(Velocity::constant{0});
        //     movement.setconstantDirection(Direction::constant{0});
        // } else if (sensorValues.lidardist[1] <
        //            DEFENCE_STOP_LINE_TRACK_LIDAR_DIST) {
        //     movement.setconstantVelocity(Velocity::constant{0});
        //     movement.setconstantDirection(Direction::constant{0});
        // }

        movement.setconstantBearing(Bearing::constant{
            0, -clipAngleto180degrees(
                   processedValues.defenceRobotAngleBisector + 180)});

        
        last_bearing = processedValues.relativeBearing;

        // movement.setconstantBearing(Bearing::constant{0.0,sensorValues.relativeBearing});

    } 

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
#endif

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

#ifdef LOCALISE_CODE
    Point location = {0, 0};
    truthValues.bearingtoField =
        clipAngleto180degrees(sensorValues.relativeBearing);
    movement.setconstantBearing(Bearing::constant{LOCALISE_CODE_TARGET_BEARING,
                                                  truthValues.bearingtoField});
    locationtime += dt;
    if (locationtime <= 5000)
        location = {0, 0};
    else if (locationtime <= 10000)
        location = {33, 49};
    else if (locationtime <= 15000)
        location = {33, -49};
    else if (locationtime <= 20000)
        location = {-33, -49};
    else
        locationtime = 0;
    movetoPoint(location);
#endif

#ifdef ATTACK_BOT_CODE


    double direction =
        clipAngleto180degrees(processedValues.ball_relativeposition.angle);

    // if (true){
    //   movement.setconstantVelocity(Velocity::constant{300});
    //   movement.setconstantDirection(Direction::constant{-170});
    // }
    // if (true){
    //   Serial.print(sensorValues.angleBisector);
    //   Serial.print(", ");
    //   movement.setlinetrackDirection(Direction::linetrack{sensorValues.depthinLine,
    //   90, 0.4, false});
    //   movement.setconstantVelocity(Velocity::constant{300});
    // }
    if (execution.setStrategy == 1) {
        if (processedValues.ball_relativeposition.distance > 50) {
            execution.strategy = 2;
            execution.setStrategy = 0;
        } else {
            execution.strategy = 2;
            execution.setStrategy = 0;
        }
    }

    // action
    // if (true){
    //     execution.setStrategy = 0;
    //     movement.setconstantVelocity(Velocity::constant{300});
    //     movement.setconstantDirection(Direction::constant{0});
    // }
    if (sensorValues.onLine == 2) {
        movement.setconstantDirection(Direction::constant{
            clipAngleto180degrees(180 + sensorValues.angleBisector)});
        movement.setconstantVelocity(Velocity::constant{350});
    }

    else if (processedValues.averageCatchmentValues >= CATCHMENT_THRESHOLD) { // 720
        execution.kickStrategySequence = 0;

        if (execution.kickComplete == 1) {
            execution.setStrategy = 1;
            execution.kickStrategySequence = 0;
            execution.kickComplete = 0;
            execution.targetBearing = 0;

        } else if (execution.strategy == 1) {
            execution.targetBearing = 0;
        }
        execution.targetBearing = 0;
        // Serial.println(processedValues.ball_relativeposition.distance);
        // Serial.println(sensorValues.ball_relativeposition.distance);
        Vector yellow_goalactualposition = {
            processedValues.yellowgoal_relativeposition.angle +
                processedValues.relativeBearing,
            processedValues.yellowgoal_relativeposition.distance};
        // Serial.print(direction);
        // Serial.print(", ");
        // Serial.println(processedValues.ball_relativeposition.distance);
        // if ((direction < 5 || direction > -5) &
        // processedValues.ball_relativeposition.distance < 32){
        //   movement.setconstantVelocity(Velocity::constant{});
        //   movement.setconstantDirection(Direction::constant{sensorValues.yellowgoal_relativeposition.angle});
        // }
        if (processedValues.ballExists == 0) {

            movetoPoint({0, -30});
        } else {
            // execution.targetBearing =  sensorValues.relativeBearing -
            // sensorValues.yellowgoal_relativeposition.angle;
            //  Serial.print("h");
            //  Serial.print(processedValues.ball_relativeposition.distance);
            //  Serial.print("h");

            Serial.print("k");
            execution.targetBearing = 0;

            movement.setconstantVelocity(
                Velocity::constant{movement.applySigmoid(
                    500, 300,
                    curveAroundBallMultiplier(
                        processedValues.ball_relativeposition.angle,
                        processedValues.ball_relativeposition.distance - 20,
                        70 - 20),
                    2.5)});
            solenoid.kick = 0;
            movement.setconstantDirection(Direction::constant{
                ballAngleOffset(processedValues.ball_relativeposition.distance,
                                processedValues.ball_relativeposition.angle) +
                processedValues.ball_relativeposition.angle});
        }
    } else if (processedValues.averageCatchmentValues <= CATCHMENT_THRESHOLD) { // 720

    #ifdef STRATEGY1

        if (processedValues.lidarDistance[0] < 30 && processedValues.lidarConfidence[0] == 0
            || millis() - execution.lastavoidTime < 1000){
            if (processedValues.lidarDistance[0] < 30 
                && processedValues.lidarConfidence[0] == 0){
                    execution.lastavoidTime = millis();
                    execution.setAvoidBotDirection == true;
                }
            
            if (processedValues.robot_position.x <=0){
                movement.setconstantDirection(Direction::constant{-100});
                movement.setconstantVelocity(Velocity::constant{600});
            }
            else{
                movement.setconstantDirection(Direction::constant{100});
                movement.setconstantVelocity(Velocity::constant{600});
            }
            execution.targetBearing =
            processedValues.relativeBearing +
            processedValues.yellowgoal_relativeposition.angle;
        }
        else if (Vector::fromPoint(processedValues.robot_position).distance > 60) {
            // analogWrite(DRIBBLER_PWM, 14);
            execution.targetBearing =
                processedValues.relativeBearing +
                processedValues.yellowgoal_relativeposition.angle;
            solenoid.kick = 1024;
            movement.setconstantVelocity(Velocity::constant{300});
            movement.setconstantDirection(Direction::constant{processedValues.yellowgoal_relativeposition.angle});
            execution.setStrategy == 1;
        } else {
            execution.targetBearing =
                processedValues.relativeBearing +
                processedValues.yellowgoal_relativeposition.angle;
            movement.setconstantVelocity(Velocity::constant{400});
            movement.setconstantDirection(Direction::constant{processedValues.yellowgoal_relativeposition.angle});
        }
    #endif

    #ifdef STRATEGY2
        if (execution.kickStrategySequence == 1) {
            if (processedValues.relativeBearing <
                    execution.targetBearing +
                        KICK_BEARING_ERROR && // NEED TO RECRAFT!!!
                processedValues.relativeBearing >
                    execution.targetBearing - KICK_BEARING_ERROR) {     
                    solenoid.kick = 1024;
            }
        } else if (processedValues.robot_position.x - KICK_POINT.x <
                       X_LOCALISATION_ERROR_THRESHOLD &&
                   processedValues.robot_position.x - KICK_POINT.x >
                       -X_LOCALISATION_ERROR_THRESHOLD &&
                   processedValues.robot_position.y - KICK_POINT.y <
                       Y_LOCALISATION_ERROR_THRESHOLD &&
                   processedValues.robot_position.y - KICK_POINT.y >
                       -Y_LOCALISATION_ERROR_THRESHOLD) {
            execution.targetBearing = (processedValues.relativeBearing +
                processedValues.yellowgoal_relativeposition.angle);
            movement.setBearingSettings(-300,300,2,0,0);
            movetoPoint(KICK_POINT);
            
            execution.kickStrategySequence = 1;
        }

        else {
            Vector localisation = {processedValues.robot_position.x, processedValues.robot_position.y};
            movetoPoint(KICK_POINT);
                execution.targetBearing = (processedValues.relativeBearing +
            processedValues.yellowgoal_relativeposition.angle);
            movement.setBearingSettings(-300,300,2,0,0);
            //movement.setBearingSettings(-100,100,1,0,0);
    
        }
    #endif
    }
    movement.setconstantBearing(Bearing::constant{
        execution.targetBearing, processedValues.relativeBearing});

#endif
    // if (distance < 10) solenoid.kick = 255;

    // Serial.println(sensorValues.ball_relativeposition.distance);
    //  movement.setconstantVelocity(Velocity::constant{400});
    // Serial.println(sensorValues.ball_relativeposition.distance);
    //  sensorfusion.updateSensorValues(movement.getmotorValues()[0],movement.getmotorValues()[1],movement.getmotorValues()[2],
    //                                movement.getmotorValues()[3], 0,
    //                                sensorValues.backlidardist, 0, 0,
    //                                localizeWithBothGoals().x(),
    //                                localizeWithBothGoals().y());
    // Vector localisation = sensorfusion.updateLocalisation();

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
    printDouble(Serial, processedValues.yellowgoal_relativeposition.angle, 3, 1);
    Serial.print(" | attackGoalDist: ");
    printDouble(Serial, processedValues.yellowgoal_relativeposition.distance, 3,
                1);
    
    Serial.print(" | defenceGoalAngle: ");
    printDouble(Serial, processedValues.bluegoal_relativeposition.angle, 3, 1);
    Serial.print(" | defenceGoalDist: ");
    printDouble(Serial, processedValues.bluegoal_relativeposition.distance, 3, 1);
    Serial.print(" | ballAngle: ");
    printDouble(Serial, processedValues.ball_relativeposition.angle, 3, 1);
    Serial.print(" | ballDist: ");
    printDouble(Serial, processedValues.ball_relativeposition.distance, 3, 1);
    // processed Values
    Serial.print(" | ballExistence: ");
    printDouble(Serial, processedValues.ballExists, 1, 1);
    Serial.print(" | attackGoal Existence: ");
    printDouble(Serial, processedValues.yellowgoal_exists, 1, 1);
    Serial.print(" | defenceGoal Existence: ");
    printDouble(Serial, processedValues.bluegoal_exists, 1, 1);
    // Location
    Serial.print(" | X_position: ");
    printDouble(Serial, processedValues.robot_position.x, 3, 0);
    Serial.print(" | Y_position: ");
    printDouble(Serial, processedValues.robot_position.y, 3, 0);
    Serial.println("");
#endif

    movement.drive({processedValues.robot_position.x, processedValues.robot_position.y});
}