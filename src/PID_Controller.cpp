#include "PID_Controller.h"

PID_Controller::PID_Controller(double p, double i, double d)
	: P(p), I(i), D(d) {}

double PID_Controller::control(double state_desired, double state_is) {

	double state_difference, control_in, proportional, differential, control_out;
	double dt = 0.001;

	// Control deviation
	state_difference = state_desired - state_is;
	control_in = state_difference;

	// PID controller
	proportional = P * control_in;
	differential = D * (control_in - control_in_before) / dt;
	integral += I * control_in * dt;
	integral = std::max(std::min(integral, 1.0), -1.0);

	// Limit output to be between -1 and 1
	control_out = std::min(std::max(proportional + differential + integral, -1.0), 1.0);
	control_in_before = control_in;

	return control_out;
}