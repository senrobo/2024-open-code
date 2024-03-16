#ifndef BALLPOSITION_H
#define BALLPOSITION_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>

#include <array>

#include "kalman.h"
#include "main.h"
#include "shared.h"
#include "util.h"
#include "vector.h"

class BallPosition {
   public:
    BallPosition();
    Vector updatePosition();
    void updateSensorMeasurement(int x_cam, int y_cam);
    void updateConstants(int dt);

   private:
    // n = no. of states
    //  m = no. of measurements aka pls kill me
    // l = 2; // no. of control inputs
    int n = 6;
    int m = 2;
    int _l = 2;

    int dt = 0;
    int cameraVariance = 10;

    int x_camera, y_camera;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n,n);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n,_l);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m,n);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n,n);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m,m);
    Eigen::MatrixXd I = Eigen::MatrixXd::Zero(n,n);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n,n);
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n,m);;

    Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(n); // Initial state estimate

    Eigen::VectorXd z = Eigen::VectorXd::Zero(m);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(_l);

    KalmanFilter kf;
};

#endif
