#include "main.h" 
#include "pros/motors.h"
#include "pros/motors.hpp"
// #include "pros/misc.h"
// #include "pros/optical.hpp" // for optical sensor
// #include "pros/rtos.hpp"
#include <cmath> // for pow() and fabs()

void initialize() {}

void disabled() {}

void competition_initialize() {}

// Controller setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Optical sensor setup
pros::Optical optical_sensor(14);

// Drivetrain setup
pros::Motor left_front(8, pros::v5::MotorGear::blue);
pros::Motor left_middle(9, pros::v5::MotorGear::blue);
pros::Motor left_back(-10, pros::v5::MotorGear::blue);

pros::Motor right_front(-1, pros::v5::MotorGear::blue);
pros::Motor right_middle(2, pros::v5::MotorGear::blue);
pros::Motor right_back(-3, pros::v5::MotorGear::blue);

pros::MotorGroup left_mg({8, 9, -10}, pros::v5::MotorGears::blue); // left motors
pros::MotorGroup right_mg({-1, 2, -3}, pros::v5::MotorGears::blue); // right motors

// Motor Setup
pros::Motor top_intake(5);
pros::Motor bottom_intake(6);

/* PID  *
 * WIP! 
 * Reference: https://www.vexforum.com/t/how-to-implement-a-pid/92878/2
 // */

void drive_pid(double target, double kp, double ki, double kd) {
    double error = 0; 
    double prev_error = 0;
    double integral = 0;
    double derivative = 0;

    const double tolerance = 5; // acceptable error in encoder ticks
    const int max_time = 3000;   // timeout in ms
    int elapsed_time = 0;

    // Reset encoders
    left_mg.tare_position();
    right_mg.tare_position();

    while (elapsed_time < max_time) {
        // Calculate average position of drivetrain
        double left_pos = left_mg.get_position();
        double right_pos = right_mg.get_position();
        double average_pos = (left_pos + right_pos) / 2.0;

        // PID calculations
        error = target - average_pos;
        integral += error;
        derivative = error - prev_error;

        // PID output
        double output = (kp * error) + (ki * integral) + (kd * derivative);

        // Move drivetrain
        left_mg.move(output);
        right_mg.move(output);

        prev_error = error;

        // Exit if within tolerance
        if (fabs(error) < tolerance) break;

        pros::delay(20);
        elapsed_time += 20;
    }

    // Stop motors
    left_mg.move(0);
    right_mg.move(0);
}

void autonomous() {
	drive_pid(1000, 0.3, 0.0, 0.2);
}

double exponential_drive(double x) {
    double sign;
    if (x >= 0) {
        sign = 1.0;
    } else {
        sign = -1.0;
    }
    double mag = fabs(x);               
    double y = 1.2 * pow(1.043, mag) - 1.2 + 0.2 * mag;
    return sign * y;                 
}

void opcontrol() {
	while (true) {
		// Left and right y inputs
		double left_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double right_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		pros::delay(20);

		// // Turn stick movement into exponetial drive function
		// double left_speed = exponential_drive(right_stick);
		// double right_speed = exponential_drive(left_stick);

		// Move motors
		left_mg.move(exponential_drive(left_stick));
		right_mg.move(exponential_drive(right_stick));
		pros::delay(20);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            top_intake.move(-127);
            bottom_intake.move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            top_intake.move(127);
            bottom_intake.move(127);
        } else {
            top_intake.move_voltage(0);
            bottom_intake.move_voltage(0);
        };

		optical_sensor.disable_gesture();
    	pros::delay(20);
	}
}