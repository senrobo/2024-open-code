#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>

#include <array>

#include "PacketSerial.h"
#include "kalman.h"
#include "shared.h"
#include "util.h"

class Sensorfusion {
  public:
    Sensorfusion();
    Vector updateLocalisation();
    void updateSensorValues(int flpwm, int frpwm, int blpwm, int brpwm,
                            int frontTof, int backTof, int leftTof,
                            int rightTof, int x_camera, int y_camera);
    void updateConstants(double frontVariance, double backVariance,
                         double leftVariance, double rightVariance,
                         double x_camVariance, double y_camVariance);

  private:
    // n = no. of states
    //  m = no. of measurements aka pls kill me
    // l = 2; // no. of control inputs
    int n = 4;
    int m = 6;
    int _l = 2;

    int dt = loopTimeinmicros() / 1000000;
    int tofVariance = 111115;
    int lidarVariance = 11115;
    int cameraVariance = 20;

    int flpwm, frpwm, blpwm, brpwm;

    int leftVectorPWMmultlier, rightVectorPWMmultlier;
    int frontTof, backTof, leftTof, rightTof;
    int x_camera, y_camera;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, _l);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, n);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
    Eigen::MatrixXd I = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n, m);
    ;

    Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(n); // Initial state estimate
    Eigen::VectorXd z = Eigen::VectorXd::Zero(m);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(_l);
    ;

    void initializeKalmanFilter();

    KalmanFilter kf;
};

#endif