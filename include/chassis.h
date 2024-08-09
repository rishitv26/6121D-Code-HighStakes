#ifndef CHASSIS_HS_H
#define CHASSIS_HS_H

#include <vector>
#include "odom.h"

class Chassis {
private:
    pros::MotorGroup* left;
    pros::MotorGroup* right;
    bool is_auton;

    Odom odom;
public:

    /**
     * @brief Construct a new Chassis object
     * 
     * Note that this is a normal tank drive (as of now)
     */
    Chassis();
    
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

    /**
     * @brief Destroy the Chassis object
     * 
     */
    ~Chassis();

};


#endif // CHASSIS_HS_H