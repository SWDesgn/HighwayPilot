#include "LongController.h"

LongitudinalController::LongitudinalController()
	: k1(0.2), k2(3.0), PID_Controller(0.0, 0.1, 0.0) {}


void LongitudinalController::control_vel(const InputData f_inputData, OutputData& f_outputData) {

	double v_desired;

	// calculate free-drive velocity
	v_desired = calc_max_speed(f_inputData);
	control_vel(f_inputData, f_outputData, v_desired);

	f_outputData.debugOut.v_desired = v_desired;
}


void LongitudinalController::control_vel(const InputData f_inputData, OutputData& f_outputData, double v_desired) {

	double a, v_diff, ctrl_in, ctrl_long_out;

	// Control difference
	v_diff = v_desired - f_inputData.veh_vel_abs;
	a = f_inputData.veh_accX;

	// Define the input of the pid controller and caltulate output
	ctrl_in = k2 * v_diff;
	ctrl_long_out = control(ctrl_in, a);

	// Use gas or breaking pedal according to the output of the controller
	if (ctrl_long_out > 0) {
		f_outputData.out_gas = ctrl_long_out;
		f_outputData.out_brake = 0.0;
	}
	else {
		f_outputData.out_gas = 0.0;
		f_outputData.out_brake = std::abs(ctrl_long_out);
	}

	// Output controller states for degugging reasons
	f_outputData.debugOut.out[0] = v_desired;
	f_outputData.debugOut.out[1] = f_inputData.veh_vel_abs;
	f_outputData.debugOut.out[2] = k2 * v_diff;
	f_outputData.debugOut.out[3] = a;
	f_outputData.debugOut.out[4] = ctrl_long_out;
}


void LongitudinalController::control_dist(const InputData f_inputData, OutputData& f_outputData) {

	double dist_is, dist_desired, v_desired, v_max, time_to_vehicle_ahead, vehicle_min_distance;

	// Parameters
	time_to_vehicle_ahead = 2.0;
	vehicle_min_distance = 6.0;

	// Follow-up mode
	dist_desired = f_inputData.veh_vel_abs * time_to_vehicle_ahead + vehicle_min_distance;
	v_max = calc_max_speed(f_inputData);

	// Check if vehicle ahead is detected
	if (f_inputData.relObj.id != -1) {
		dist_is = f_inputData.relObj.rel_dist[0];
		v_desired = -k1 * (dist_desired - dist_is) + f_inputData.relObj.abs_vel;
		v_desired = std::min(v_desired, v_max);
		f_outputData.debugOut.out[5] = dist_is;
	}
	else {
		v_desired = v_max;
		f_outputData.debugOut.out[5] = 0.0;
	}

	// Control velocity
	control_vel(f_inputData, f_outputData, v_desired);

	f_outputData.debugOut.dist_desired = dist_desired;
	f_outputData.debugOut.v_desired = v_max;
}


double  LongitudinalController::calc_max_speed(const InputData f_inputData) {

	// Maximum speed if not otherwise given
	double v_desired = 130 / 3.6;

	if (f_inputData.road_speed_limit > 0 && f_inputData.road_speed_limit < v_desired) {
		v_desired = f_inputData.road_speed_limit; //already in m/s
	}

	//a_lat = v^2/R
	double latAcc = (v_desired * v_desired) * std::abs(f_inputData.road_curvature);

	if (latAcc > ACC_Y_MAX) {
		v_desired = std::sqrt(ACC_Y_MAX / std::abs(f_inputData.road_curvature));
	}

	return v_desired;
}