#include "main.h"
#include <Servo.h>
#include <Arduino.h>
#include "SPI.h"
#include <array>

#include "PacketSerial.h"
#include "ballposition.h"
#include "kalman.h"
#include "movement.h"
#include "sensorfusion.h"
#include "shared.h"
#include "config.h"

//hello


// put function declarations here:

Point KICK_POINT{0, 0};

Sensorfusion sensorfusion;
BallPosition ballposition;
PacketSerial L3TeensySerial;
PacketSerial L1Serial;
PacketSerial L2toL1Serial;
Movement movement;

SensorValues sensorValues;
Solenoid solenoid;
TruthValues truthValues;
Servo frontDrib;
Execution execution;

// https://www.desmos.com/calculator/5uexflvu3o

int highValues[36] = {
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
    5.00, 5.00, 5.00, 5.00, 5.00, 5.00};

double maxRecordedValue[36]{
881 , 882 , 863 , 870 , 874 , 864 , 857 , 871 , 868 , 862 , 861 , 873 , 866 , 871 , 879 , 872 , 870 , 890 , 881 , 873 , 
869 , 881 , 869 , 883 , 891 , 891 , 864 , 887 , 876 , 869 , 877 , 952 , 874 , 880 , 876 , 866 
};

double minRecordedValue[36]{
859 , 865 , 835 , 854 , 846 , 836 , 839 , 844 , 850 , 836 , 833 , 840 , 857 , 850 , 857 , 858 , 864 , 868 , 864 , 853 , 854 , 
857 , 851 , 860 , 872 , 868 , 847 , 866 , 856 , 850 , 860 , 950 , 860 , 866 , 855 , 847 
};

double calculatedthesholdValue[36]{
  834, 833, 818, 822, 828, 821, 826, 820,   5, 821, 832, 822, 830, 832, 825, 841, 825, 832, 838, 834,
  829, 841, 843, 840, 850, 846, 848, 843, 836, 840, 830, 831, 828, 844, 823, 827 
};


double frontMirrorMapping(double distance) {
    if (distance != 500) {
        return (5.39617219 * powf(10, -7) * powf(distance, 5) - 1.61827053 * powf(10, -4) * powf(distance, 4) + 1.88452202 * powf(10, -2) * powf(distance, 3) - 1.04993865 * powf(10, 0) * powf(distance, 2) + 2.82679699 * 10 * distance - 2.83240269 * powf(10, 2));
    } else
        return 0;
}
// 4.32382133e-04 -5.61664545e-02  2.35497365e+00 -6.84446262e-02
double backMirrorMapping(double distance) {
    if (distance != 500) {
        return 3.3654176 * powf(10, -4) * powf(distance, 3) - 4.28607791 * powf(10, -2) * powf(distance, 2) + 1.92452824 * distance - 1.02719597 * 0.1;
    } else
        return 0;
}

double ballMirrorMapping(double distance) {
    if (distance != 500) {
        return (2.72234207 * powf(10, -7) * powf(distance, 5) - 7.95782065 * powf(10, -5) * powf(distance, 4) + 9.11543654 * powf(10, -3) * powf(distance, 3) - 4.96659050 * powf(10, -1) * powf(distance, 2) + 1.32335351 * 10 * distance - 1.23966075 * powf(10, 2));
    } else
        return 0;
}

Vector localizeWithOffensiveGoal(double angle) {
    // Compute a "fake" center vector from the goal vector
    const Vector realGoalToCenter = {-angle, -90};
    const auto fakeCenter = sensorValues.yellowgoal_relativeposition + realGoalToCenter;
    Vector actualCenter = {clipAngleto180degrees(fakeCenter.angle + 180), fakeCenter.distance};
    return actualCenter;
};

void receiveL3TxData(const byte *buf, size_t size) {
    // load payload
    L3Txpayloaf payload;
    // if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));
    sensorValues.relativeBearing = -payload.sensorvalues.relativeBearing;
    sensorValues.yellowgoal_relativeposition = payload.sensorvalues.yellowgoal_relativeposition;
    sensorValues.ball_relativeposition = payload.sensorvalues.ball_relativeposition;
    sensorValues.bluegoal_relativeposition = payload.sensorvalues.bluegoal_relativeposition;

    for (int i = 0; i < 4; i++) {
        sensorValues.lidardist[i] = payload.sensorvalues.lidardist[i];
    }
    sensorValues.ball_relativeposition.angle += 0;
    sensorValues.yellowgoal_relativeposition.distance = frontMirrorMapping(sensorValues.yellowgoal_relativeposition.distance);
    sensorValues.bluegoal_relativeposition.distance = frontMirrorMapping(sensorValues.bluegoal_relativeposition.distance);
    sensorValues.ball_relativeposition.distance = ballMirrorMapping(sensorValues.ball_relativeposition.distance);
    return;
}
L1TxPayload L1payload;

void receiveL1TxData(const byte *buf, size_t size){
  //load payload

  //if (size != sizeof(payload)) return;
  memcpy(&L1payload, buf, sizeof(L1payload));
  sensorValues.angleBisector = clipAngleto180degrees(360 - L1payload.l1TxData.angleBisector);
  sensorValues.onLine = L1payload.l1TxData.onLine;
  sensorValues.depthinLine = L1payload.l1TxData.depthinLine;
  sensorValues.ballinCatchment = L1payload.l1TxData.ballinCatchment;
  sensorValues.linetrackldr1 = L1payload.l1TxData.linetrackldr1;
  sensorValues.linetrackldr2 = L1payload.l1TxData.linetrackldr2;
  // sensorValues.SerialLDRID = L1payload.l1TxData.SerialLDRID;
  // sensorValues.SerialLDRvalue = L1payload.l1TxData.SerialLDRvalue;
  return;
}

void driveBrushless(double MINPWM, double MAXPWM, int dt = 100) {
    if (MAXPWM >= MINPWM) {
        for (int i = MINPWM; i <= MAXPWM; i = i + 1) {
            analogWrite(DRIBBLERPWM, i);
            Serial.println(i);
            delay(dt);
        }
    } else {
        for (int i = MAXPWM; i <= MINPWM; i = i - 1) {
            analogWrite(DRIBBLERPWM, i);
            Serial.println(i);
            delay(dt);
        }
    }
}

static double loopTimeinMillis() {
    timeControl.now = millis();
    timeControl.dt = timeControl.now - timeControl.last;
    timeControl.last = timeControl.now;
    return timeControl.dt;
}

Vector localize() {
    if ((sensorValues.yellowgoal_relativeposition.distance < 90 && processedValues.yellowgoal_exists == 1) || (processedValues.yellowgoal_exists == 1 && processedValues.bluegoal_exists == 0)) {
        sensorfusion.updateSensorValues(movement.getmotorValues()[0], movement.getmotorValues()[1], movement.getmotorValues()[2],
                                        movement.getmotorValues()[3], sensorValues.lidardist[0], sensorValues.lidardist[2],
                                        sensorValues.lidardist[3], sensorValues.lidardist[1],
                                        localizeWithOffensiveGoal().x(), localizeWithOffensiveGoal().y());
        Vector localisation = sensorfusion.updateLocalisation();
        return localisation;
    } else if ((sensorValues.bluegoal_relativeposition.distance < 90 && processedValues.bluegoal_exists == 1) || (processedValues.bluegoal_exists == 1 && processedValues.yellowgoal_exists == 0)) {
        sensorfusion.updateSensorValues(movement.getmotorValues()[0], movement.getmotorValues()[1], movement.getmotorValues()[2],
                                        movement.getmotorValues()[3], sensorValues.lidardist[0], sensorValues.lidardist[2],
                                        sensorValues.lidardist[3], sensorValues.lidardist[1],
                                        localizeWithDefensiveGoal().x(), localizeWithDefensiveGoal().y());
        Vector localisation = sensorfusion.updateLocalisation();
        return localisation;
    } else {
        sensorfusion.updateSensorValues(movement.getmotorValues()[0], movement.getmotorValues()[1], movement.getmotorValues()[2],
                                        movement.getmotorValues()[3], sensorValues.lidardist[0], sensorValues.lidardist[2],
                                        sensorValues.lidardist[3], sensorValues.lidardist[1],
                                        localizeWithBothGoals().x(), localizeWithBothGoals().y());
        Vector localisation = sensorfusion.updateLocalisation();
        return localisation;
    }
    // sensorfusion.updateSensorValues(movement.getmotorValues()[0],movement.getmotorValues()[1],movement.getmotorValues()[2],
    //                                   movement.getmotorValues()[3], 0, sensorValues.lidardist[3], 0, 0,
    //                                   localizeWithDefensiveGoal().x(), localizeWithDefensiveGoal().y());
}

void movetoPoint(Point destination) {
    Vector localisation = localize();
    double distance = (localisation - Vector::fromPoint(destination)).distance;
    movement.setconstantVelocity(Velocity::constant{movement.applySigmoid(400, 30, (distance) / 20, 1.3)});
    movement.setmovetoPointDirection(Direction::movetoPoint{localisation, destination});
}

ProcessedValues processedValues;

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
        previous_averageballinCatchment = averageballinCatchment / averageballinCatchmentCount;
        averageballinCatchmentCount = 0;
        averageballinCatchment = 0;
        return previous_averageballinCatchment;
    }
}

void setup() {
  delay(10);
  analogWriteResolution(10);
  Serial5.begin(115200);
  Serial3.begin(115200);
  Serial.begin(9600);
  L3TeensySerial.setStream(&Serial5); // set serial stream to packet communcation to default serial
  L3TeensySerial.setPacketHandler(&receiveL3TxData);

  L1Serial.setStream(&Serial3);
  L1Serial.setPacketHandler(&receiveL1TxData);
  // Wire.begin();
  // Wire.setClock(10000);
  // bno.begin(0x4A, Wire);
  //setReports();
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

void loop() {
  #ifdef DEBUG_THRESHOLD_VALUES
  highValues[sensorValues.linetrackldr2] = sensorValues.linetrackldr1;
  for (int i = 0 ; i < 36; i++){
    calculatedthesholdValue[i] = minRecordedValue[i] + (maxRecordedValue[i] - minRecordedValue[i]) * 0.5;
    if (i == 35) {
      Serial.print(calculatedthesholdValue[i]);
      Serial.println(" ");
    }
    else {
      Serial.print(calculatedthesholdValue[i]);
      Serial.print(" , ");
    }
  }
  #endif
  // SETUP PHASE
  analogWrite(DRIBBLER_PWM_PIN, BRUSHLESS_DEFAULT_SPEED);

  L3TeensySerial.update();
  L1Serial.update();
  L3TeensySerial.update();
  L1Serial.update();
  verifyingObjectExistance();
  processLidars();

  double dt = loopTimeinMillis();
  ballposition.updateConstants(dt/1000);
  ballposition.updateSensorMeasurement(sensorValues.ball_relativeposition.x(),sensorValues.ball_relativeposition.y());
  processedValues.ball_relativeposition = ballposition.updatePosition();


  sensorfusion.updateConstants(frontVariance, backVariance, leftVariance, rightVariance, 10, 15);

  #ifdef DEBUG_LIGHT_RING
  const auto printSerial = [](int value){
    Serial.printf("%3d",value);
  };
  for (int i = 0; i < 36; i++){
    if (i == 35) {
    printSerial(highValues[i]);
    Serial.println(" ");
    }
    else {
    printSerial(highValues[i]);
    Serial.print(" , ");
    }
  }
  #endif

  #ifdef DEFENCE_BOT_CODE

  if (sensorValues.onLine == 1 && processedValues.ballExists == 1){
    if (clipAngleto180degrees(sensorValues.angleBisector) <= 90 &&
      clipAngleto180degrees(sensorValues.angleBisector) >= -90 ){
        processedValues.defenceRobotAngleBisector = clipAngleto180degrees(180 + sensorValues.angleBisector);
        processedValues.defenceRobotDepthinLine = (1.0 - sensorValues.depthinLine) + 1.0;
    }
    else{
      processedValues.defenceRobotAngleBisector = sensorValues.angleBisector;
      processedValues.defenceRobotDepthinLine = sensorValues.depthinLine;
    }

    // if (true){
    //   movement.setlinetrackDirection(Direction::linetrack{processedValues.defenceRobotDepthinLine,
    //                           processedValues.defenceRobotAngleBisector, DEFENCE_DEPTH_IN_LINE, true});
    //   //movement.setconstantDirection(Direction::constant{90});
    //   // double progress = (sensorValues.bluegoal_relativeposition.angle -
    //   //                   (processedValues.ball_relativeposition.angle - 180))/90;
    //   // movement.setconstantVelocity(Velocity::constant{movement.applySigmoid(300,100, progress, DEFENCE_ACCELERATION_MULTIPLIER)});
    //   movement.setconstantVelocity(Velocity::constant{400});
    // }
    if (processedValues.ballExists == 0){
      movement.setconstantVelocity(Velocity::constant{0});
      movement.setconstantDirection(Direction::constant{0});
    }
    else if (clipAngleto360degrees(sensorValues.bluegoal_relativeposition.angle) > 
      clipAngleto360degrees(processedValues.ball_relativeposition.angle - 180) && sensorValues.lidardist[3] >= DEFENCE_STOP_LINE_TRACK_LIDAR_DIST){
      movement.setlinetrackDirection(Direction::linetrack{processedValues.defenceRobotDepthinLine,
                              processedValues.defenceRobotAngleBisector, DEFENCE_DEPTH_IN_LINE, false});
      double progress = (sensorValues.bluegoal_relativeposition.angle -
                        (processedValues.ball_relativeposition.angle - 180))/90;
      movement.setconstantVelocity(Velocity::constant{movement.applySigmoid(400,200, progress, DEFENCE_ACCELERATION_MULTIPLIER)});
      //movement.setconstantVelocity(Velocity::constant{400});
            }

    else if (clipAngleto360degrees(sensorValues.bluegoal_relativeposition.angle) < 
      clipAngleto360degrees(processedValues.ball_relativeposition.angle - 180) && sensorValues.lidardist[1] >= DEFENCE_STOP_LINE_TRACK_LIDAR_DIST){
      movement.setlinetrackDirection(Direction::linetrack{processedValues.defenceRobotDepthinLine,
                      processedValues.defenceRobotAngleBisector, DEFENCE_DEPTH_IN_LINE, true}); 

      double progress = (-clipAngleto360degrees(sensorValues.bluegoal_relativeposition.angle) +
                        clipAngleto360degrees(processedValues.ball_relativeposition.angle - 180))/90;
      movement.setconstantVelocity(Velocity::constant{movement.applySigmoid(400,200, progress, DEFENCE_ACCELERATION_MULTIPLIER)});
      //movement.setconstantVelocity(Velocity::constant{400});
    }

    else if (sensorValues.lidardist[3] < DEFENCE_STOP_LINE_TRACK_LIDAR_DIST){
      movement.setconstantVelocity(Velocity::constant{0});
      movement.setconstantDirection(Direction::constant{0});
    }
    else if(sensorValues.lidardist[1] < DEFENCE_STOP_LINE_TRACK_LIDAR_DIST){
      movement.setconstantVelocity(Velocity::constant{0});
      movement.setconstantDirection(Direction::constant{0});
    }

  movement.setconstantBearing(Bearing::constant{0,-clipAngleto180degrees(processedValues.defenceRobotAngleBisector + 180)});
  //movement.setconstantBearing(Bearing::constant{0.0,sensorValues.relativeBearing});
  
  }
  else{
    movement.setconstantVelocity(Velocity::constant{0});
    movement.setconstantBearing(Bearing::constant{0,0});
  }
  //movement.setconstantBearing(Bearing::constant{0.0,0});


  #ifdef DEBUG_DEFENCE_BOT
  const auto printSerial = [](double value){
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



  //Serial.println(sensorValues.ballinCatchment);

  //Serial.println(processedValues.ball_relativeposition.angle);



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
  Point location = {0,0};
  truthValues.bearingtoField = clipAngleto180degrees(sensorValues.relativeBearing);
  movement.setconstantBearing(Bearing::constant{LOCALISE_CODE_TARGET_BEARING,truthValues.bearingtoField});
  locationtime += dt;
  if (locationtime <= 5000) location = {0,0};
  else if (locationtime <= 10000) location = {33, 49};
  else if (locationtime <= 15000) location = {33,-49};
  else if (locationtime <= 20000) location = {-33,-49};
  else locationtime = 0;
  movetoPoint(location);
  #endif


  #ifdef ATTACK_BOT_CODE

  truthValues.bearingtoField = clipAngleto180degrees(sensorValues.relativeBearing);
  double direction =clipAngleto180degrees(processedValues.ball_relativeposition.angle);
  
  // if (true){
  //   movement.setconstantVelocity(Velocity::constant{300});
  //   movement.setconstantDirection(Direction::constant{-170});
  // }
  // if (true){    
  //   Serial.print(sensorValues.angleBisector);
  //   Serial.print(", ");
  //   movement.setlinetrackDirection(Direction::linetrack{sensorValues.depthinLine, 90, 0.4, false});
  //   movement.setconstantVelocity(Velocity::constant{300});
  // }
  if (execution.setStrategy == 1){
    if (processedValues.ball_relativeposition.angle > 135 || processedValues.ball_relativeposition.angle < - 135){
      execution.strategy = 2;
      execution.setStrategy = 0;
    }
    else{
      execution.strategy = 2;
      execution.setStrategy = 0;
      }
  }


  // action
  if (sensorValues.onLine > 0){
    movement.setconstantDirection(Direction::constant{clipAngleto180degrees(180+sensorValues.angleBisector)});
    movement.setconstantVelocity(Velocity::constant{350});

  }
  else if (avg_ballinCatchment(dt,10) >= 0){ //720
  
    if (execution.kickComplete == 1){
      execution.setStrategy = 1;
      execution.kickStrategySequence = 0;
      execution.kickComplete = 0;
      execution.targetBearing = 0;
    }
    else if (execution.strategy == 1){
      execution.targetBearing = 0;
    }
    //Serial.println(processedValues.ball_relativeposition.distance);
    execution.targetBearing = 0;
    // Serial.println(sensorValues.ball_relativeposition.distance);
    Vector yellow_goalactualposition = {sensorValues.yellowgoal_relativeposition.angle + sensorValues.relativeBearing, sensorValues.yellowgoal_relativeposition.distance};
    // Serial.print(direction);
    // Serial.print(", ");
    // Serial.println(processedValues.ball_relativeposition.distance);
    // if ((direction < 5 || direction > -5) & processedValues.ball_relativeposition.distance < 32){
    //   movement.setconstantVelocity(Velocity::constant{});
    //   movement.setconstantDirection(Direction::constant{sensorValues.yellowgoal_relativeposition.angle});
    // }
    if (processedValues.ballExists == 0){

      movetoPoint({0,-30});
    }
    else {
      //execution.targetBearing =  sensorValues.relativeBearing - sensorValues.yellowgoal_relativeposition.angle;
      // Serial.print("h");
      // Serial.print(processedValues.ball_relativeposition.distance);
      // Serial.print("h");


      movement.setconstantVelocity(Velocity::constant{movement.applySigmoid(450,250,
        curveAroundBallMultiplier(processedValues.ball_relativeposition.angle, processedValues.ball_relativeposition.distance - 20,
        70 - 20), 0.7)});
      solenoid.kick = 0;
      movement.setconstantDirection(Direction::constant{ballAngleOffset(processedValues.ball_relativeposition.distance, processedValues.ball_relativeposition.angle) 
                                                      + processedValues.ball_relativeposition.angle});
      //execution.targetBearing = - sensorValues.yellowgoal_relativeposition.angle;
      //execution.targetBearing = 0;
    }
  }
  else if (avg_ballinCatchment(dt, 10) <= 680) {  // 720
        if (execution.strategy == 1) {
            if (localizeWithOffensiveGoal().distance > 60) {
                // analogWrite(DRIBBLER_PWM, 14);
                solenoid.kick = 1024;
                movement.setconstantVelocity(Velocity::constant{300});
                movement.setconstantDirection(Direction::constant{0});
                execution.setStrategy == 1;
            } else {
                execution.targetBearing = sensorValues.relativeBearing - sensorValues.yellowgoal_relativeposition.angle;
                movement.setconstantVelocity(Velocity::constant{400});
                movement.setconstantDirection(Direction::constant{0});
            }
        } 
        else if (execution.strategy == 2) {
            if (execution.kickStrategySequence = 1) {
                if (sensorValues.relativeBearing < execution.targetBearing + KICK_BEARING_ERROR &&  // NEED TO RECRAFT!!!
                    sensorValues.relativeBearing > execution.targetBearing - KICK_BEARING_ERROR) {
                    if (avg_ballinCatchment(dt, 10) > 680) {
                        solenoid.kick = 1024;
                        execution.kickComplete = 1;
                    }
                }
            } else if (localize().x() - KICK_POINT.x < X_LOCALISATION_ERROR_THRESHOLD &&
                       localize().x() - KICK_POINT.x > -X_LOCALISATION_ERROR_THRESHOLD &&
                       localize().y() - KICK_POINT.y < Y_LOCALISATION_ERROR_THRESHOLD &&
                       localize().y() - KICK_POINT.y > -Y_LOCALISATION_ERROR_THRESHOLD) {
                execution.targetBearing = 50;
                movement.setconstantVelocity(Velocity::constant{0});
                execution.kickStrategySequence = 1;
            }

            else {
                Vector localisation = localize();
                movetoPoint(KICK_POINT);
            }
        }
      }

  
  movement.setconstantBearing(Bearing::constant{execution.targetBearing, sensorValues.relativeBearing});

  #endif
  //if (distance < 10) solenoid.kick = 255;

  //Serial.println(sensorValues.ball_relativeposition.distance);
  // movement.setconstantVelocity(Velocity::constant{400});
  //Serial.println(sensorValues.ball_relativeposition.distance);
  // sensorfusion.updateSensorValues(movement.getmotorValues()[0],movement.getmotorValues()[1],movement.getmotorValues()[2],
  //                               movement.getmotorValues()[3], 0, sensorValues.backlidardist, 0, 0, 
  //                               localizeWithBothGoals().x(), localizeWithBothGoals().y());
  //Vector localisation = sensorfusion.updateLocalisation();

  #ifdef DEBUG_EVERYTHING

  //lightRing
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
  //L3 Data
  Serial.print(" | bearing: ");
  printDouble(Serial, sensorValues.relativeBearing, 3, 1);
  Serial.print(" | frontLidar: ");
  printDouble(Serial, sensorValues.lidardist[0], 3, 0);
  Serial.print(" | rightLidar: ");
  printDouble(Serial, sensorValues.lidardist[1], 3, 0);
  Serial.print(" | backLidar: ");
  printDouble(Serial, sensorValues.lidardist[2], 3, 0);
  Serial.print(" | leftLidar: ");
  printDouble(Serial, sensorValues.lidardist[3], 3, 0);
  Serial.print(" | attackGoalAngle: ");
  printDouble(Serial, sensorValues.yellowgoal_relativeposition.angle, 3, 1);
  Serial.print(" | attackGoalDist: ");
  printDouble(Serial, sensorValues.yellowgoal_relativeposition.distance, 3, 1);
  Serial.print(" | defenceGoalAngle: ");
  printDouble(Serial, sensorValues.bluegoal_relativeposition.angle, 3, 1);
  Serial.print(" | defenceGoalDist: ");
  printDouble(Serial, sensorValues.bluegoal_relativeposition.distance, 3, 1);
  Serial.print(" | ballAngle: ");
  printDouble(Serial, sensorValues.ball_relativeposition.angle, 3, 1);
  Serial.print(" | ballDist: ");
  printDouble(Serial, sensorValues.ball_relativeposition.distance, 3, 1);
  //processed Values
  Serial.print(" | ballExistence: ");
  printDouble(Serial, processedValues.ballExists,1, 1);
  Serial.print(" | attackGoal Existence: ");
  printDouble(Serial, processedValues.yellowgoal_exists,1, 1);
  Serial.print(" | defenceGoal Existence: ");
  printDouble(Serial, processedValues.bluegoal_exists,1, 1);
  Serial.print(" | frontlidarConf: ");
  printDouble(Serial, processedValues.lidarConfidence[0],1, 0);
  Serial.print(" | rightlidarConf: ");
  printDouble(Serial, processedValues.lidarConfidence[1],1, 0);
  Serial.print(" | backlidarConf: ");
  printDouble(Serial, processedValues.lidarConfidence[2],1, 0);
  Serial.print(" | leflidarConf: ");
  printDouble(Serial, processedValues.lidarConfidence[3],1, 0);
  Serial.print(" | processedfrontLidar: ");
  printDouble(Serial, processedValues.lidarDistance[0], 3, 0);
  Serial.print(" | processedrightLidar: ");
  printDouble(Serial, processedValues.lidarDistance[1], 3, 0);
  Serial.print(" | processedbackLidar: ");
  printDouble(Serial, processedValues.lidarDistance[2], 3, 0);
  Serial.print(" | processedleftLidar: ");
  printDouble(Serial, processedValues.lidarDistance[3], 3, 0);
  Serial.println("");
  #endif

  

  movement.drive({0,0});
  solenoid.kick = 0;
  byte buf[sizeof(L2TxtoL1payload)];
  memcpy(buf,&solenoid, sizeof(solenoid));
  L1Serial.send(buf,sizeof(buf));
}