#ifndef ODOM_HS_H
#define ODOM_HS_H

// TODO: Improve getTheta and getThetaRadians using IMU sensor.
class Odom {
private:
    OdomState state;
    std::shared_ptr<okapi::OdomChassisController> odomChassis;
public:
    /**
     * @brief Construct a new Odom object
     */
    Odom();

    /**
     * @brief Initialize the odom object with the given parameters.
     * 
     * all settings specified in the settings.h
     */
    void initialize();

    /**
     * @brief Updates the X, Y, and theta coordinates
     * 
     * not neccessary to call before use. Get methods call this beforehand
     */
    void update();

    /**
     * @brief get the latest X coordinate
     * 
     * @return X value
     */
    double getX();

    /**
     * @brief get the latest Y coordinate
     * 
     * @return Y value
     */
    double getY();

    /**
     * @brief get the latest theta angle in degrees
     * 
     * @return Theta angle as degrees
     */
    double getTheta();

    /**
     * @brief get the latest theta angle in radians 
     * 
     * @return Theta angle as radians
     */
    double getThetaRadians();

};



#endif // ODOM_HS_H