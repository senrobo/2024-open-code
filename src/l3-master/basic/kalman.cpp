#include "kalman.h"

KalmanFilter::KalmanFilter(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q,
                           Eigen::MatrixXd R, Eigen::MatrixXd I, Eigen::MatrixXd H) 
                           : _A(A), memberB(B), _Q(Q), _R(R), _I(I), _H(H), n(A.rows()), 
                             m(H.rows()), _l(B.cols()) // why is x_hat and p(n,n) necessary
                           {
                            //I.setIdentity();
        
                           }
void KalmanFilter::updateConstants(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q,
                                   Eigen::MatrixXd R, Eigen::MatrixXd I, Eigen::MatrixXd H) {
    _A = A;
    memberB = B;
    _Q = Q;
    _R = R;
    _I = I;
    _H = H;
}
void KalmanFilter::initialize(Eigen::VectorXd x_hat, Eigen::MatrixXd P, Eigen::MatrixXd K) {
    _x_hat = x_hat;
    memberP = P;
    _K = K;
}

void KalmanFilter::predict(Eigen::VectorXd u) {
    auto printMatrix = [](Eigen::MatrixXd x, int row, int col) {
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                Serial.print(x(i, j));
                Serial.print(" ");
            }
            Serial.println(" ");
        }
        Serial.println(" ");
    };
    auto printVector = [](Eigen::VectorXd x, int row) {
        for (int i = 0; i < row; i++) {
            Serial.print(x(i));
            Serial.print(" ");
            Serial.println(" ");
        }
        Serial.println(" ");
    };
    _x_hat = _A * _x_hat + memberB * u;  // apriori state estimate
    // Eigen::MatrixXd G = A * P * A.transpose();
    // printMatrix(G,n,n);
    // printMatrix(P,n,n);

    memberP = _A * memberP * _A.transpose() + _Q;
    // P = P + Q;
}

void KalmanFilter::correction(Eigen::VectorXd z) {
    _z = z;
    auto printMatrix = [](Eigen::MatrixXd x, int row, int col) {
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                Serial.print(x(i, j));
                Serial.print(" ");
            }
            Serial.println(" ");
        }
        Serial.println(" ");
    };

    _K = memberP * _H.transpose() * (_H * memberP * _H.transpose() + _R).inverse();

    _x_hat = _x_hat + _K * (_z - _H * _x_hat);  // aposteriori state estimate

    memberP = (_I - _K * _H) * memberP;
}

Eigen::MatrixXd KalmanFilter::updateState() {
    return _x_hat;
}