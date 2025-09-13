#include <regex>
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#include "main.h" 
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath> // for pow() and fabs()

const double TRACK_WIDTH = 315;

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

// PID variables
// Constants
double kp = 0.3;
double ki = 0.0;
double kd = 0.0;

// Other variables
// Straight drive
int error; // sensor value - target value (ositional value)
int prev_error = 0; // position 20 ms ago
int derivative; // error - prev_error (calculates speed)
int total_error = 0; // total_error + error 

// Turning
double turn_error = 0;
double turn_prev_error = 0;
double turn_derivative = 0;
double turn_integral = 0;

bool pid_enabled = true;

double mmToMotorDegrees(double distance_mm) {
    const double wheel_diameter = 100.0; // change if different wheel
    const double wheel_circumference = wheel_diameter * M_PI;
    const double gear_ratio = 48.0 / 36.0; // wheel / motor
    return (distance_mm / wheel_circumference) * 360.0 * gear_ratio;
}

void turnAngle (int degrees, int max_time_ms, int velocity) {
    left_mg.move_velocity(velocity);
    right_mg.move_velocity(velocity);
    // Calculate the arc length for the turn
    double turn_circumference = M_PI * TRACK_WIDTH;
    double distance_mm = (turn_circumference * degrees) / 360.0;
    double target = mmToMotorDegrees(distance_mm);

    // Reset PID state
    double error = 0;
    double prev_error = 0;
    double derivative = 0;
    double integral = 0;

    // Reset motor encoders before turning
    left_mg.tare_position();
    right_mg.tare_position();

    int start_time = pros::millis();
    while (true) {
        double left_pos = left_mg.get_position();
        double right_pos = right_mg.get_position();
        double avg_pos = (left_pos - right_pos) / 2.0; // Turning: difference

        error = target - avg_pos;
        derivative = error - prev_error;
        integral += error;

        double motor_power = (kp * error) + (ki * integral) + (kd * derivative);

        // Clamp motor power to velocity limit
        if (motor_power > velocity) motor_power = velocity;
        if (motor_power < -velocity) motor_power = -velocity;

        left_mg.move(motor_power);
        right_mg.move(-motor_power);

        prev_error = error;

        if (fabs(error) < 10) break;
        if (pros::millis() - start_time > max_time_ms) break;

        pros::delay(20);
    }
    left_mg.move(0);
    right_mg.move(0);
}



void driveDistance(int target, int max_time_ms, int velocity) {
    left_mg.move_velocity(velocity);
    right_mg.move_velocity(velocity);
    target = mmToMotorDegrees(target);
    // Reset PID state
    double error = 0;
    double prev_error = 0;
    double derivative = 0;
    double integral = 0;
    // Reset motor encoders before driving
    left_mg.tare_position();
    right_mg.tare_position();

    int start_time = pros::millis();
    while (true) {
        // Average drivetrain position
        double left_pos = left_mg.get_position();
        double right_pos = right_mg.get_position();
        double avg_pos = (left_pos + right_pos) / 2.0;

        // PID calculations)
        error = target - avg_pos;
        derivative = error - prev_error;
        integral += error;

        double motor_power = (kp * error) + (ki * integral) + (kd * derivative);

        // Clamp motor power to velocity limit
        if (motor_power > velocity) motor_power = velocity;
        if (motor_power < -velocity) motor_power = -velocity;

        // Apply to drivetrain
        left_mg.move(motor_power);
        right_mg.move(motor_power);

        prev_error = error;

        // Exit conditions
        if (fabs(error) < 10) break;  // Close enough to target
        if (pros::millis() - start_time > max_time_ms) break;  // Timeout

        pros::delay(20);
    }
    // Stop drivetrain
    left_mg.move(0);
    right_mg.move(0);
}

void autonomous() {
    driveDistance(750, 1000,50);
    pros::delay(250);
    turnAngle(90, 1000,50);
}

double exponential_drive(double x) {
    double sign = (x >= 0) ? 1.0 : -1.0;
    double abs_x = fabs(x);
    double y = 1.2 * pow(1.043, abs_x) - 1.2 + 0.2 * abs_x;
    return sign * y;
}

bool tounge_state = false;

void opcontrol() {
    left_mg.move_velocity(100);
    right_mg.move_velocity(100);
	while (true) {
		// Left and right y inputs
		double left_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double right_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// Apply exponential drive function
		double left_speed = exponential_drive(left_stick);
		double right_speed = exponential_drive(right_stick);

		// Move motors
		left_mg.move(left_speed);
		right_mg.move(right_speed);
		pros::delay(20);

        //L2 INtAKE
        // l1 outtake
        // r1 top
        // r2 middle 

        // MIDDLE
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            top_intake.move(127);
            bottom_intake.move(-127);
            bottom_back_intake.move(-127);
        // OUTTAKE
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            top_intake.move(127);
            bottom_intake.move(127);
            bottom_back_intake.move(127);
        // INTAKE
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            top_intake.move(-127);
            bottom_intake.move(127);
            bottom_back_intake.move(-127);
        // TOP
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            top_intake.move(127);
            bottom_intake.move(127);
            bottom_back_intake.move(-127);
        } else {
            top_intake.move_voltage(0);
            bottom_intake.move_voltage(0);
            bottom_back_intake.move_voltage(0);
        };
        

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