#ifndef VECTOR_H
#define VECTOR_H

#include <iostream>

struct Point {
    double x;
    double y;

    bool operator==(const Point &other) const {
        return {x == other.x && y == other.y};
    }

    bool operator!=(const Point &other) const {
        return {!(*this == other)};
    }
};

// Vector in polar coordinates with angles in degrees and distance in cm
class Vector {
   public:
    Vector();
    Vector(double angle, double distance);

    static Vector fromPoint(Point point);

    double angle;
    double distance;

    double x() const;
    double y() const;

    // operators
    Vector &operator=(const Vector &other);
    Vector operator+(const Vector &other) const;
    Vector operator-() const;  // negation
    Vector operator-(const Vector &other) const;
    Vector operator*(const double other) const;  // angle is not affected
    Vector operator/(const double other) const;  // angle is not affected
    bool exists() const;
};

#endif