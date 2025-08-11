#include "main.h" 
#include "pros/misc.h"

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

// Controller setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor setup
pros::MotorGroup left_mg({11, -12, -13}); // left motors
pros::MotorGroup right_mg({-1, 2, 3}); // right motors

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
	}
}