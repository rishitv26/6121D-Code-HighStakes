#include "odom.h"
#include "api.h"

Odom::Odom(pros::Rotation *right, pros::Rotation *left, pros::Rotation *center, pros::Imu *imu)
{
    initialize(right, left, center, imu);
}

Odom::Odom() {}

void Odom::initialize(pros::Rotation *r, pros::Rotation *l, pros::Rotation *c, pros::Imu *i)
{
    right = r;
    left = l;
    center = c;
    imu = i;
}

void Odom::update()
{
    // TODO
}

double Odom::getX()
{
    return X;
}

double Odom::getY()
{
    return Y;
}

double Odom::getTheta()
{
    return theta;
}
