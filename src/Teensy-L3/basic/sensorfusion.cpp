#include "sensorfusion.h"

Sensorfusion::Sensorfusion() : kf(A, B, Q, R, I, H) {
    kf.updateConstants(A, B, Q, R, I, H);
    kf.initialize(x_hat, P, K);
};

void Sensorfusion::updateConstants(double frontVariance, double backVariance,
                                   double leftVariance, double rightVariance,
                                   double x_camVariance, double y_camVariance) {
    A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

    /*
    assuming max rpm is 900, diameter of wheel is 52mm
    units in cm/s
    therefore max speed is assumed to be 240cm/s
    input assumed to be in fl - br, fr - bl

    angle between wheel and x-axis is 34degrees

    measurements in 121.5 - x_fronttof, - (121.5 - x_backtof) , 91 - y_righttof,
    - (91 - y_lefttof), x_camera, y_camera
    */

    B << sind(56) * dt, sind(56) * dt, sind(34) * dt, sind(34) * dt, sind(56),
        sind(56), sind(34), sind(34);

    H << 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0;

    Q << 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20;
    // assuming error +- 0.25mm (according to VL53L1x datasheet) is standard
    // deviation and error is Gaussian

    R << rightVariance, 0, 0, 0, 0, 0, 0, leftVariance, 0, 0, 0, 0, 0, 0,
        frontVariance, 0, 0, 0, 0, 0, 0, backVariance, 0, 0, 0, 0, 0, 0,
        x_camVariance, 0, 0, 0, 0, 0, 0, y_camVariance;

    P << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    I << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    // Serial.print(dt);
    kf.updateConstants(A, B, Q, R, I, H);
}

void Sensorfusion::updateSensorValues(int flpwm, int frpwm, int blpwm,
                                      int brpwm, int frontTof, int backTof,
                                      int leftTof, int rightTof, int x_camera,
                                      int y_camera) {
    //     u << leftVectorPWMmultlier, rightVectorPWMmultlier;

    //     //values from https://www.desmos.com/calculator/wnmxj2vywo

    //      rightVectorPWMmultlier = constrain(0.1879461 * ((flpwm + brpwm) / 2)
    //      - 22.7294, 0, 0.1879461 * ((flpwm + brpwm) / 2) - 22.7294);
    //      leftVectorPWMmultlier = constrain(0.1879461 * ((frpwm + blpwm) / 2)
    //      - 22.7294, 0, 0.1879461 * ((flpwm + brpwm) / 2) - 22.7294);
    u << 0, 0;

    x_hat << 0, 0, 0, 0;
    // kf.initialize(x_hat, P, z);
    z << 91 - rightTof, leftTof - 91, 121.5 - frontTof, backTof - 121.5,
        x_camera, y_camera;
    kf.predict(u);
    kf.correction(z);
}

Vector Sensorfusion::updateLocalisation() {
    Eigen::VectorXd state = kf.updateState();

    Point location;
    location.x = state(0);
    location.y = state(1);
    Vector locationvector = Vector::fromPoint(location);
    return locationvector;
}
