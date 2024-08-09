#include "odom.h"
#include "api.h"
#include "settings.h"
#include "okapi/api.hpp"
using namespace okapi;

Odom::Odom() {}

void Odom::initialize()
{
    odomChassis = ChassisControllerBuilder()
        .withMotors(LEFT_PORTS, RIGHT_PORTS)
        // green gearset, 4 inch wheel diameter, 11.5 inch wheel track
        .withDimensions(AbstractMotor::gearset::green, {{DRIVETRAIN_WHEEL_INCHES, WHEEL_TRACK_INCHES}, imev5GreenTPR})
        .withSensors(
            RotationSensor(LEFT_TRACKING), // left sensor port in settings
            RotationSensor(RIGHT_TRACKING, true),  // right sensor port in settings
            RotationSensor(CENTER_TRACKING)  // middle sensor port in settings
        )
        // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
        // specify the middle encoder distance (1 in) and diameter (2.75 in)
        .withOdometry({{TRACKING_DIAMETER, PARALLEL_SENSOR_TRACK_WIDTH, MIDDLE_ENCODER_DISTANCE, TRACKING_DIAMETER}, 360}, StateMode::CARTESIAN)
        .buildOdometry();
}

void Odom::update()
{
    state = odomChassis->getState();
}

double Odom::getX()
{
    update();
    return state.x.convert(inch);
}

double Odom::getY()
{
    update();
    return state.y.convert(inch);
}

double Odom::getTheta()
{
    update();
    return state.theta.convert(degree);
}

double Odom::getThetaRadians()
{
    update();
    return state.theta.convert(radian);
}
