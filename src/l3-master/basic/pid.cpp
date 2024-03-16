#include "pid.h"

#include <Arduino.h>

PIDController::PIDController(double setpoint, double min, double max,
                             double kp, double kd, double ki, double maxi, double dt, double measurementgain) : _setpoint(setpoint), _min(min), _max(max), _kp(kp), _kd(kd), _ki(ki), _maxi(maxi), _dt(dt), _measurementgain(measurementgain){};

double PIDController::advance(double processvariable) {
    _error = _setpoint - processvariable;

    const auto now = micros();
    const auto dt = now - _lasttime;
    _lasttime = now;

    if (dt < _dt) return _lastoutput;

    // low pass filter to reduce noise
    _currentestimate = (_measurementgain * _previousestimate) + (1 - _measurementgain) * _error;
    _previousestimate = _currentestimate;

    // PID calculation
    _integral += _lasterror * _dt;
    _integral = constrain(_integral, -_maxi, _maxi);  // integral anti-windup method
    double P = _kp * _error;
    double I = _ki * _integral * _dt;
    double D = _kd * (_currentestimate - _lasterror);
    double output = constrain(P + I + D, _min, _max);

    _lastoutput = output;
    _lasterror = _error;

    // debug
    _lastP = P;
    _lastI = I;
    _lastD = D;
    _lastinput = processvariable;
    return output;
};

void PIDController::reset() {
    _currentestimate = 0;
    _previousestimate = 0;
    _integral = 0;
    _lasterror = 0;
    _lastoutput = 0;
};

void PIDController::updateGains(double kp, double kd, double ki, double measurementgain) {
    _kp = kp;
    _kd = kd;
    _ki = ki;
    _measurementgain = measurementgain;
};

void PIDController::updateSetpoint(double setpoint) {
    _setpoint = setpoint;
};

double PIDController::getSetpoint() {
    return _setpoint;
};

void PIDController::updateLimits(double min, double max, double maxi) {
    _min = min;
    _max = max;
    _maxi = maxi;
};

void PIDController::debugController() {
    auto printserial = [](double value) {
        Serial.printf("%4d.%2d", (int)value, abs((int)(value * 100) % 100));  // 2 d.p.
    };
    // void printSerial(double value){
    //     Serial.printf("%4d.%2d", (int)value , abs((int)(value * 100) % 100));
    // }
    // how do lambda functions work?
    // auto add = [](int a, int b) -> int { return a + b; };
    // int sum = add(1, 2);  // sum will be 3

    Serial.print("Setpoint: ");
    printserial(_setpoint);
    Serial.print(" | Input: ");
    printserial(_lastinput);
    Serial.print(" | Integral: ");
    printserial(_integral);
    Serial.print(" | P: ");
    printserial(_lastP);
    Serial.print(" | I: ");
    printserial(_lastI);
    Serial.print(" | D: ");
    printserial(_lastD);
    Serial.print(" | Error: ");
    printserial(_lasterror);
    Serial.print(" | Output: ");
    printserial(_lastoutput);
    Serial.println(" ");
};
