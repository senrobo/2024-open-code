#ifndef SHARED_H
#define SHARED_H

#include "vector.h"


struct TimeControl {
    double now = 0;
    double last = 0;
    double dt = 0;
};

static TimeControl timeControl;

static double loopTimeinmicros() {
    timeControl.now = micros();
    timeControl.dt = timeControl.now - timeControl.last;
    timeControl.last = timeControl.now;
    return timeControl.dt / 1000000;
};

static double loopTimeinMillis() {
    timeControl.now = millis();
    timeControl.dt = timeControl.now - timeControl.last;
    timeControl.last = timeControl.now;
    return timeControl.dt;
}

#endif