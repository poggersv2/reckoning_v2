//#include <regex>
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#include "main.h" 
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath> // for pow() and fabs()

//#include "autonomous.h"

void initialize() {}

void disabled() {}

void competition_initialize() {}
// Controller setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Optical sensor setup
// pros::Optical optical_sensor(14);

// Drivetrain setup
// Top stacked motors should be reversed (negative number)

pros::MotorGroup left_mg({20, -19, -18}, pros::v5::MotorGears::blue); // left motor group
pros::MotorGroup right_mg({-11, 12, 13}, pros::v5::MotorGears::blue); // right motor group

// Motor Setup
pros::Motor top_intake(10);
pros::Motor bottom_intake(9);
pros::Motor bottom_back_intake(-6);

pros::adi::Pneumatics tounge('A', false); // blocker piston



double exponential_drive(double x) {
    double sign = (x >= 0) ? 1.0 : -1.0;
    double abs_x = fabs(x);
    double y = 1.2 * pow(1.043, abs_x) - 1.2 + 0.2 * abs_x;
    return sign * y;
}

bool tounge_state = false;


void intake_bang_bang(int top_intake_desired, int bottom_intake_desired, int bottom_back_intake_desired, int threshold){
    // Bang-bang for top intake
    int top_error = top_intake_desired - top_intake.get_actual_velocity();
    if (std::abs(top_error) > threshold) {
        top_intake.move((top_error > 0) ? 127 : -127);
    } else {
        top_intake.move(0);
    }

    // Bang-bang for bottom intake
    int bottom_error = bottom_intake_desired - bottom_intake.get_actual_velocity();
    if (std::abs(bottom_error) > threshold) {
        bottom_intake.move((bottom_error > 0) ? 127 : -127);
    } else {
        bottom_intake.move(0);
    }

    // Bang-bang for bottom back intake
    int back_error = bottom_back_intake_desired - bottom_back_intake.get_actual_velocity();
    if (std::abs(back_error) > threshold) {
        bottom_back_intake.move((back_error > 0) ? 127 : -127);
    } else {
        bottom_back_intake.move(0);
    }
}

void opcontrol() {
    // Slew rate variables
    double prev_left = 0;
    double prev_right = 0;
    const double slew_step = 8; // max change per loop (adjust as needed, 127/8 = ~16 loops full swing)

	while (true) {
		// Left and right y inputs
		double left_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double right_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// Apply exponential drive function
		double left_speed = exponential_drive(left_stick);
		double right_speed = exponential_drive(right_stick);

        // Slew rate limiting
        double delta_left = left_speed - prev_left;
        if (fabs(delta_left) > slew_step) {
            prev_left += slew_step * ((delta_left > 0) ? 1 : -1);
        } else {
            prev_left = left_speed;
        }

        double delta_right = right_speed - prev_right;
        if (fabs(delta_right) > slew_step) {
            prev_right += slew_step * ((delta_right > 0) ? 1 : -1);
        } else {
            prev_right = right_speed;
        }

		// Move motors with slew-limited values
		left_mg.move(prev_left);
		right_mg.move(prev_right);
		pros::delay(20);

        //L2 INtAKE
        // l1 outtake
        // r1 top
        // r2 middle 

        // MIDDLE
        // Use bang-bang intake control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_bang_bang(127, -127, -127, 10);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake_bang_bang(127, 127, 127, 10);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake_bang_bang(-127, 127, -127, 10);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_bang_bang(127, 127, -127, 10);
        } else {
            intake_bang_bang(0, 0, 0, 10);
        }
        
		// .disable_gesture();
    	pros::delay(20);
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            tounge_state = true;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            tounge_state = false;
        }

        tounge.set_value(tounge_state);


		pros::delay(20);
	}
}