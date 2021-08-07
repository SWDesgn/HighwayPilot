#pragma once
#include "Input.h"
#include "Output.h"
#include "PID_Controller.h"
#include <algorithm>
#include <cmath>

/** @file LatController.h
 *  @brief Longitudinal controller for controlling the longitudinal position of the vehicle.
 *  Two driving scenarios can be used, namely follow-up mode or free driving.
 *  Therefore a suitable activation level of the gas pedal is chosen based on the current state of the vehicle.
 *  @author Lukas Roming und Oliver Altergott
 */

 //! Lateral controller 
class LongitudinalController : PID_Controller {

public:
	LongitudinalController();

	/** @brief Controls the velocity of the vehicle to be at the maximum allowed speed.
	 *  @param f_inputData data from the input object including the states of the vehicle
	 *  @param f_outputData reference to an object that outputs data for debugging purposes
	 *  @return void
	 */
	void control_vel(const InputData f_inputData, OutputData& f_outputData);

	/** @brief Controls the velocity of the vehicle to be as given.
	 *  @param f_inputData data from the input object including the states of the vehicle
	 *  @param f_outputData reference to an object that outputs data for debugging purposes
	 *  @param v_desired desired velocity
	 *  @return void
	 */
	void control_vel(const InputData f_inputData, OutputData& f_outputData, double v_desired);

	/** @brief Controls the longitudinal distance to the vehicle ahead for follow-up mode.
	 *  @param f_inputData data from the input object including the states of the vehicle
	 *  @param f_outputData reference to an object that outputs data for debugging purposes
	 *  @return void
	 */
	void control_dist(const InputData f_inputData, OutputData& f_outputData);

	/** @brief Calculates the maximum allowed speed at the current position for free drining.
	 *  @param f_inputData data from the input object
	 *  @return maximum allowed speed
	 */
	double calc_max_speed(const InputData f_inputData);

private:
	double k1;		/**< factor that defines the proportionality constant between distance deviation and desired speed */
	double k2;		/**< factor that defines the proportionality constant between speed deviation and desired acceleration */
};