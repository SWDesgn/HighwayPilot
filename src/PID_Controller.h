#pragma once
#include "Input.h"
#include "Output.h"
#include <algorithm>
#include <cmath>

/** @file PID_Controller.h
 *  @brief pid controller that is used to control the actual state to be as desired.
 *  Therefore, a proportional, an integral and a differential part can be used.
 *  @author Lukas Roming
 */

 //! PID controller 
class PID_Controller {

public:
	PID_Controller(double p, double i, double d);
	
	/** @brief controls the actual state to be as desired
	 *  @param state_desired desired state
	 *  @param state_is actual state
	 *  @return double controller output
	 */
	double control(double state_desired, double state_is);

private:
	double P;						/**< proportional part */ 
	double I;						/**< integral part */ 
	double D;						/**< differential part */ 
	double control_in_before = 0;	/**< input at the time step before for calculating the differential part */ 
	double integral = 0;			/**< value of the integrator for calculating the integral part */ 
};