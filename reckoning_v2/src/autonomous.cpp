#include "autonomous.h"
#include "main.h"
// #include "pros/misc.h"
// #include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// PID variables and constants
const double TRACK_WIDTH = 315;

// Constants
double kp = 0.3;
double ki = 0.0;
double kd = 0.0;

// Other variables
// Straight drive
int error; // sensor value - target value (positional value)
int prev_error = 0; // position 20 ms ago
int derivative; // error - prev_error (calculates speed)
int total_error = 0; // total_error + error 

// Turning
double turn_error = 0;
double turn_prev_error = 0;
double turn_derivative = 0;
double turn_integral = 0;

bool pid_enabled = true;

// Externs for variables/constants from main.cpp
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;

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
