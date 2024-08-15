#ifndef SETTINGS_HS_H
#define SETTINGS_HS_H

// Port deffinitions:
#define LEFT_PORTS {1, 2, 3}
#define RIGHT_PORTS {4, 5, 6}

// Odom definitions: (imu can be found out using static method)
#define RIGHT_TRACKING 69
#define LEFT_TRACKING 69
#define CENTER_TRACKING 69
#define TRACKING_DIAMETER 2.75
#define PARALLEL_SENSOR_TRACK_WIDTH 7
#define MIDDLE_ENCODER_DISTANCE 1

// Drivetrain definitions:
#define DRIVETRAIN_WHEEL_INCHES 3.25
#define WHEEL_TRACK_INCHES 11.5 // This is the space between the right WHEELS and left WHEELS.


#endif // SETTINGS_HS_H