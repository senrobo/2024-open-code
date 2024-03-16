#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>

#include <vector>

#include "pid.h"
#include "util.h"
#include "vector.h"

namespace Direction {
struct constant {
    double value;
};
struct movetoPoint {
    Vector robotCoordinate;
    Point destination;
};
struct linetrack {
    double lineDepth;
    double angleBisector;
    double targetLineDepth = 0.5;
    bool trackLeftwards = true;  // track left of an angle bisector from the
                                 // line facing into the field
};
}  // namespace Direction

namespace Velocity {
struct constant {
    double value;
};
struct stopatPoint {
    double errordistance;
    double minSpeed;
    double maxSpeed;
};
}  // namespace Velocity

namespace Bearing {
struct constant {
    double targetValue;
    double actualBearing;
};
struct moveBearingtoPoint {
    Vector robotCoordinate;
    Point destination;
    double finalBearing;
};
}  // namespace Bearing

class Movement {
   public:
    Movement();

    void updateParameters(double actualbearing, double actualdirection, double actualvelocity);

    void initialize();
    // set relavent parameters
    void setconstantDirection(Direction::constant params);
    void setmovetoPointDirection(Direction::movetoPoint params);
    void setlinetrackDirection(Direction::linetrack params);

    void setconstantVelocity(Velocity::constant params);
    void setstopatPointVelocity(Velocity::stopatPoint params);

    void setconstantBearing(Bearing::constant params);
    void setmoveBearingtoPoint(Bearing::moveBearingtoPoint params);
    void setBearingSettings(double min, double max, double KP, double KD, double KI);

    //PID Controllera
    PIDController bearingController = PIDController(0.0, -600, 600 , 3, 
                                     20,0.0 , infinity(), 1, 0.2);
    
    PIDController directionController = PIDController(0.0, -90, 90, 1.5, 
                                    0, 0, 1.0, 1, 0.2); 
    
    PIDController stopatPointController = PIDController(0.0, -360, 360 , 1.5, 
                                    0.01, 0.1, 1.0, 1, 0.2); //not yet implemented

    PIDController linetrackController = PIDController(0.0, -0.3, 0.3, 1, 
                                    0, 0, infinity(), 1, 0.2); //not yet implemented
                        
                            
    void drive(Point robotPosition);
    double applySigmoid(double startSpeed,double endSpeed, double progress, double constant);


    std::vector<double> getmotorValues();

   private:
    // parameters
    double _targetdirection;
    double _targetbearing;
    double _targetvelocity;

    //
    double _movingdirection;
    double _movingbearing;
    double _movingvelocity;

    // past values
    double _lastdirection;
    double _lastbearing;
    double _lastvelocity;

    // for Bearing::moveBearingtoPoint and Direction::movetoPoint
    Point _finalDestination;
    double _initialbearing;
    double _finalbearing;
    Vector _initialrobotcoordinate;

    // actual parameters
    double _actualvelocity;
    double _actualbearing;
    double _actualdirection;
};

#endif