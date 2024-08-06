#ifndef ODOM_HS_H
#define ODOM_HS_H


class Odom {
private:
    double X;
    double Y;
    double theta;

    pros::Rotation* right;
    pros::Rotation* left;
    pros::Rotation* center;
    pros::Imu* imu;
public:
    /**
     * @brief Construct a new Odom object
     * 
     * @param right the pointer to right tracking wheel
     * @param left the pointer to left tracking wheel
     * @param center the pointer to center tracking wheel
     * @param imu the pointer to IMU
     */
    Odom(pros::Rotation* right, pros::Rotation* left, pros::Rotation* center, pros::Imu* imu);

    /**
     * @brief Construct a new Odom object
     * 
     * If this method is called, initialize() MUST be called.
     */
    Odom();

    /**
     * @brief Initialize the odom object with the given parameters.
     * 
     * @param right the pointer to right tracking wheel
     * @param left the pointer to left tracking wheel
     * @param center the pointer to center tracking wheel
     * @param imu the pointer to IMU
     */
    void initialize(pros::Rotation* right, pros::Rotation* left, pros::Rotation* center, pros::Imu* imu);

    /**
     * @brief Updates the X, Y, and theta coordinates
     * 
     * Must get the X, Y and theta coords using the getting methods.
     */
    void update();

    /**
     * @brief get the X coordinate since the latest update call
     * 
     * @return X value
     */
    double getX();

    /**
     * @brief get the Y coordinate since the latest update call
     * 
     * @return Y value
     */
    double getY();

    /**
     * @brief get the theta coordinate since the latest update call
     * 
     * @return Theta value
     */
    double getTheta();

};



#endif // ODOM_HS_H