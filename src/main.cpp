#include "main.h"

// #include "settings.h"
// #include "chassis.h"

// Chassis chassis;

void initialize() {
	pros::lcd::initialize();
	// chassis.initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	// chassis.stopAllMotors();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor one(1);
	pros::Motor two(2);
	pros::Motor three(3);

	one.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	two.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	three.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

	while (true) {
		// chassis.op_move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		
		int scale = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		one.move(scale);
		two.move(scale);
		three.move(scale);

		std::cout << (one.get_efficiency() + two.get_efficiency() + three.get_efficiency()) / 3.0 << std::endl;

		// other stuff.. TODO (based on robor)
		pros::delay(20);
	}
	
}