#ifndef CHASSIS_HS_H
#define CHASSIS_HS_H

#include <vector>
#include "odom.h"

class Chassis {
private:
    pros::MotorGroup* left;
    pros::MotorGroup* right;
    pros::Imu* imu;
    
    pros::Rotation* r;
    pros::Rotation* l;
    pros::Rotation* c;

    bool is_auton;

    Odom odom;
public:

    /**
     * @brief Construct a new Chassis object
     * 
     * @param right_ports the right ports of robot
     * @param left_ports the left ports of robot
     * @param right_tracking the right tracking wheel
     * @param left_tracking the left tracking wheel
     * @param center_tracking center tracking wheel
     * 
     * Note that this is a normal tank drive (as of now)
     * The constructor creates motor and rotation objects in the heap, but does not do anything else.
     */
    Chassis(std::vector<std::int8_t> right_ports, std::vector<std::int8_t> left_ports, 
            std::int8_t right_tracking, std::int8_t left_tracking, std::int8_t center_tracking);
    
    /**
     * @brief Initializes all the motors and rotations in this class as neccessary. 
     */
    void initialize();

    /**
     * @brief Stops all running chassis motors. 
     * 
     * Braking will depend on the brake mode of motors defined during initialize()
     */
    void stopAllMotors();

    /**
     * @brief Moves the chassis based on analog values from opcontrol.
     * 
     * As of now, PID or other algorithms do not stabalize this, it is purely driven
     * by the driver.
     * 
     * @param X the X analog value [-127, 127]
     * @param Y the Y analog value [-127, 127]
     */
    void op_move(std::int32_t X, std::int32_t Y);

};


#endif // CHASSIS_HS_H