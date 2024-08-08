#include "odom.h"
#include "api.h"
#include "okapi/api.hpp"

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
    auto odomChassis = ChassisControllerBuilder()
        .withMotors({1, 2}, {-3, -4})
        .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
        .withOdometry() // Use default state mode and update period
        .buildOdometry(); // Return an OdometryChassisController
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
