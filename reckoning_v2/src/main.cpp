#include "main.h"
#include "pros/adi.hpp"
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
pros::MotorGroup left_mg({-11, 12, 13}); // left motors
pros::MotorGroup right_mg({1, -2, -3}); // right motors
pros::Motor bottom_intake(10); // bottom intake
pros::Motor top_intake(9); // top intake

// Pneumatics setup
pros::adi::Pneumatics blocker('A', false); // blocker piston
pros::adi::Pneumatics ramp('B', false);    // ramp piston


void autonomous() {
	// Drive forward at full speed for 1 second, then stop
	left_mg.move(-50);
	right_mg.move(-80);
	pros::delay(800);
	//blocker.set_value(true);
	//top_intake.move(127);
	
	left_mg.move(0);
	right_mg.move(0);
}

void opcontrol() {
	// State vars
	bool blocker_state = false;
	bool ramp_state = false;
	bool colour_sort_state = false;
	bool top_velocity_toggle = false;
	bool bottom_velocity_toggle = false;
	const int DEADBAND = 0;

	// --- Exponential Drive Function (WIP) ---
	/** auto exponential_drive = [](double x) {
		if (x > 127) x = 127;
		if (x < -127) x = -127;
		double sign = (x >= 0) ? 1.0 : -1.0;
		x = std::abs(x);
		double y = 1.2 * pow(1.043, x) - 1.2 + 0.2 * x;
		return y * sign;
	};
	**/

	// --- Main Loop ---
	while (true) {
		// ================= DRIVE =================
		// Left and right Y inputs
		double left_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double right_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// Reverse inputs
		left_stick = -left_stick;
		right_stick = -right_stick;

		// Apply deadband
		if (std::abs(left_stick) < DEADBAND) left_stick = 0;
		if (std::abs(right_stick) < DEADBAND) right_stick = 0;

		// Move motors
		left_mg.move(left_stick);
		right_mg.move(right_stick);
		pros::delay(20);

		// ================= INTAKE & SORTER =================
		double R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		double R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool L1 = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
		bool L2 = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
		bool X = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);


		bool UP = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
		bool DOWN = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);

		// --- Intake velocity toggles ---
		if (UP) top_velocity_toggle = !top_velocity_toggle;
		if (DOWN) bottom_velocity_toggle = !bottom_velocity_toggle;
		int top_velocity = top_velocity_toggle ? 100 : 50;
		int bottom_velocity = bottom_velocity_toggle ? 100 : 50;

		// --- Colour sorter toggle ---
		if (X) colour_sort_state = !colour_sort_state;

		// --- Intake logic ---
		if (colour_sort_state) {
			bottom_intake.move_velocity(100);
			bottom_intake.move(-127);
			top_intake.move_velocity(100);
			top_intake.move(127);
		} else {
			if (R1) {
				bottom_intake.move_velocity(bottom_velocity);
				bottom_intake.move(127);
				top_intake.move_velocity(top_velocity);
				top_intake.move(127);
				pros::delay(20);
			} else if (R2) {
				bottom_intake.move_velocity(bottom_velocity);
				bottom_intake.move(-127);
				top_intake.move_velocity(top_velocity);
				top_intake.move(-127);
				pros::delay(20);
			} else {
				bottom_intake.move(0);
				top_intake.move(0);
				pros::delay(20);
			}
		}

		// ================= PNEUMATICS =================
		if (L2) {
			blocker_state = !blocker_state;
			blocker.set_value(blocker_state);
		}
		pros::delay(20);
		if (L1) {
			ramp_state = !ramp_state;
			ramp.set_value(ramp_state);
		}
		pros::delay(20);
	}
}