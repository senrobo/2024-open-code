#include "ballposition.h"


BallPosition::BallPosition() : kf(A, B, Q, R, I, H){
  kf.updateConstants(A, B, Q, R, I, H);
  kf.initialize(x_hat, P, K);
 };

void BallPosition::updateConstants(int dt){
  A << 1, 0, dt, 0, dt * dt, 0,
       0, 1, 0, dt, 0, dt * dt,
       0, 0, 1, 0, dt, 0,
       0, 0, 0, 1, 0, dt,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0;


  B << 0, 0,
       0, 0,
       0, 0,
       0, 0,
       0, 0,
       0, 0;

  H << 1, 0, dt, 0, powf(dt, 2), 0,
       0, 1, 0, dt, 0, powf(dt, 2);

  Q << 0.01, 0, 0, 0, 0, 0,
       0, 0.01, 0, 0, 0, 0,
       0, 0, .01, 0, 0, 0,
       0, 0, 0, .01, 0, 0,
       0, 0, 0, 0, .5, 0,
       0, 0, 0, 0, 0, .5,


  R << cameraVariance, 0,
       0, cameraVariance;
  
  P << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
  
  I << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
 

  kf.updateConstants(A,B,Q,R,I,H);

}

void BallPosition::updateSensorMeasurement(int x_cam, int y_cam) {
    u << 0, 0;

    // values from https://www.desmos.com/calculator/wnmxj2vywo

    x_hat << 0, 0, 0, 0, 0, 0;
    // kf.initialize(x_hat, P, z);
    z << x_cam, y_cam;
    kf.predict(u);
    kf.correction(z);
}

Vector BallPosition::updatePosition() {
    Eigen::VectorXd state = kf.updateState();

    Point location;
    location.x = state(0);
    location.y = state(1);

    Vector locationvector = Vector::fromPoint(location);

    return locationvector;
}
