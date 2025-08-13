#include "main.h" 
#include "pros/misc.h"
#include "pros/optical.hpp"

void initialize() {}

void disabled() {}

void competition_initialize() {}

// Controller setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Optical sensor setup
pros::Optical optical_sensor(14);

// Drivetrain setup
pros::MotorGroup left_mg({11, -12, -13}); // left motors
pros::MotorGroup right_mg({-1, 2, 3}); // right motors

// Motor Setup


void autonomous() {}

void opcontrol() {
	while (true) {
		// Left and right y inputs
		double left_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double right_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		pros::delay(20);

		// Move motors
		left_mg.move(left_stick);
		right_mg.move(right_stick);
		pros::delay(20);

		optical_sensor.disable_gesture();
    	pros::delay(20);
	}
}