#ifndef PID_H
#define PID_H
#include <Arduino.h>

#include "vector.h"

class PIDController {
  public:
    PIDController(double setpoint, double min, double max, double kp, double kd,
                  double ki, double maxi, double dt, double measurementgain);

    double advance(double processvariable);
    void reset();

    void updateGains(double kp, double kd, double ki, double measurementgain);
    void updateSetpoint(double setpoint);
    double getSetpoint();
    void updateLimits(double min, double max, double maxi);

    void debugController();

  private:
    // tuning parameters
    double _setpoint;
    double _min;
    double _max;
    double _kp;
    double _ki;
    double _kd;
    double _maxi; // prevents system saturation
    double _dt;
    // internal values
    double _integral;
    double _error;
    double _lasterror;
    double _lastoutput;
    double _lasttime;
    double _currentestimate;
    double _previousestimate;
    double _measurementgain;

    // debug values
    double _lastinput;
    double _lastP;
    double _lastD;
    double _lastI;
};

#endif