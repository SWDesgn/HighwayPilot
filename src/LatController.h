#pragma once
#include "Input.h"
#include "Output.h"
#include "PID_Controller.h"
#include <Log.h>
#include <cmath>

/** @file LatController.h
 *  @brief Lateral controller for controlling the lateral position of the vehicle to stay on the desired lane.
 *  Therefore a suitable steering angle is chosen based on the current state of the vehicle.
 *  @author Lukas Roming
 */

//! Lateral controller 
class LatController : PID_Controller {

public:
	LatController();

	/** @brief controls the lateral position of the vehicle.
	 *  @param f_inputData data from the input object including the states of the vehicle
	 *  @param f_outputData reference to an object that outputs data for debugging purposes
	 *  @param lane desired lane (-1:right 0:middle 1:left)
	 *  @return void
	 */
	void control_lane_deviation(const InputData f_inputData, OutputData& f_outputData, ELane f_lane);

private:
	double k;					/**< factor that defines the influence of lane deviation on the desired vehicle orientation */ 
	double time_back_on_lane;	/**< desired time the vehicle takes in order to equalize the current lane deviation */
	double smoothed_steering;
};