#include "chassis.h"
#include "settings.h"
#include "api.h"

Chassis::Chassis(): is_auton(false) {}

void Chassis::initialize()
{
    left = new pros::MotorGroup(LEFT_PORTS);
    right = new pros::MotorGroup(RIGHT_PORTS);
}

void Chassis::stopAllMotors()
{
    left->brake();
    right->brake();
}

void Chassis::op_move(std::int32_t X, std::int32_t Y)
{
}

Chassis::~Chassis()
{
    delete left;
    delete right;
}








