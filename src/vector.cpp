#include "vector.h"

#include <Arduino.h>

#include <cmath>

#include "util.h"

Vector::Vector() {}

Vector::Vector(double angle, double distance)
    : angle(angle), distance(distance) {}

Vector Vector::fromPoint(Point point) {
    return {atand(point.x, point.y),
            sqrtf(point.x * point.x + point.y * point.y)};
}

double Vector::x() const { return sind(angle) * distance; }

double Vector::y() const { return cosd(angle) * distance; }

Vector &Vector::operator=(const Vector &other) {
    this->angle = other.angle;
    this->distance = other.distance;
    return *this;
}

Vector Vector::operator+(const Vector &other) const {
    const double x =
        distance * sind(angle) + other.distance * sind(other.angle);
    const double y =
        distance * cosd(angle) + other.distance * cosd(other.angle);
    return {atand(x, y), sqrtf(x * x + y * y)};
}

Vector Vector::operator-() const {
    return {clipAngleto180degrees(angle + 180), distance};
}

Vector Vector::operator-(const Vector &other) const {
    const double x =
        distance * sind(angle) - other.distance * sind(other.angle);
    const double y =
        distance * cosd(angle) - other.distance * cosd(other.angle);
    return {atand(x, y), sqrtf(x * x + y * y)};
}
Vector Vector::operator*(const double other) const {
    return {angle, distance * other};
};
Vector Vector::operator/(const double other) const {
    return {angle, distance / other};
};
bool Vector::exists() const {
    return !std::isnan(angle) && !std::isnan(distance);
}