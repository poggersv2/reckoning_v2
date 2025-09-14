#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void autonomous();

// PID variables and constants
extern const double TRACK_WIDTH;
extern double kp, ki, kd;
extern int error, prev_error, derivative, total_error;
extern double turn_error, turn_prev_error, turn_derivative, turn_integral;
extern bool pid_enabled;

#ifdef __cplusplus
}
#endif