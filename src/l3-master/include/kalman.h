#ifndef KALMAN_H
#define KALMAN_H

#include <ArduinoEigenDense.h>

class KalmanFilter {
   public:
    KalmanFilter(
        Eigen::MatrixXd _A,       // system dynamic coefficient matrix
        Eigen::MatrixXd memberB,  // control input coefficient matrix
        Eigen::MatrixXd _Q,       // process noise covariance
        Eigen::MatrixXd _R,       // measurement noise covariance
        Eigen::MatrixXd _I,       // identity matrix
        Eigen::MatrixXd _H        // measurement coefficient matrix
    );

    void updateConstants(
        Eigen::MatrixXd _A,       // system dynamic coefficient matrix
        Eigen::MatrixXd memberB,  // control input coefficient matrix
        Eigen::MatrixXd _Q,       // process noise covariance
        Eigen::MatrixXd _R,       // measurement noise covariance
        Eigen::MatrixXd _I,       // identity matrix
        Eigen::MatrixXd _H        // measurement coefficient matrix
    );

    void initialize(Eigen::VectorXd _x_hat, Eigen::MatrixXd memberP, Eigen::MatrixXd K);

    void predict(Eigen::VectorXd _u);
    void correction(Eigen::VectorXd _z);

    Eigen::MatrixXd updateState();

   private:
    // parameters
    Eigen::MatrixXd _A, memberB, _Q, _R, _I, _H;

    //internal variables
    Eigen::MatrixXd memberP; // apriori and aposteriori estimate error
    Eigen::MatrixXd _K; // kalman gain
    
    //matrix dimensions
    int m, n, _l;


    // measurement
    Eigen::VectorXd _u;  // control input
    Eigen::VectorXd _z;  // measurement
    // output
    Eigen::VectorXd _x_hat;

    long long time;
};

#endif