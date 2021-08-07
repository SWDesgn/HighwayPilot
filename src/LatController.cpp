#include "LatController.h"

LatController::LatController()
	: k(0.3), smoothed_steering(0.0), time_back_on_lane(0.5), PID_Controller(10.0, 1.0, 0.0) {}


void LatController::control_lane_deviation(const InputData f_inputData, OutputData& f_outputData, ELane f_lane) {

	double a, angle_is, angle_desired, deviation_laneCenter, ctrl_lat_out;
	
	// Parameter
	a = 0.01;

	// Deviation from lane
	switch (f_lane) {
	case ELane::RIGHT_LANE:
			deviation_laneCenter = f_inputData.egoLane.dev_dist - f_inputData.rightLane.dev_dist;
			angle_is = f_inputData.rightLane.dev_angl;
			break;
	case ELane::LEFT_LANE:
			deviation_laneCenter = f_inputData.egoLane.dev_dist - f_inputData.leftLane.dev_dist;
			angle_is = f_inputData.leftLane.dev_angl;
			break;
	case ELane::MIDDLE_LANE:
		default:
			deviation_laneCenter = f_inputData.egoLane.dev_dist;
			angle_is = f_inputData.egoLane.dev_angl;
			break;
	}

	// Define pid controller inputs (deviation/(v*t) = GK/HYP = sin(alpha) ~ angle_desired)
	angle_desired = -k * deviation_laneCenter / (1.0 + f_inputData.veh_vel_abs * time_back_on_lane);
	ctrl_lat_out = control(angle_desired, angle_is);

	smoothed_steering = smoothed_steering - a * (smoothed_steering - ctrl_lat_out);

	// Set steering angle and output debug data
	f_outputData.out_steering = smoothed_steering;
	f_outputData.debugOut.dist_ego_centerLane = deviation_laneCenter;
	f_outputData.debugOut.out[9] = deviation_laneCenter;
}
