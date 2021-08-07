#pragma once
#include<Log.h>

/** @file Output.h
 *  @brief HighwayPilot output data definition
 *  @author Oliver Altergott
 */


 //! Debug outputs
struct DebugData {

	/// desired and ego velocity 
	double v_desired, v_ego;

	/// desired deviation distances 
	double dist_desired, dist_ego_obj, dist_ego_centerLane;

	/// outputs for IPGControl plotting
	double out[10];

	/// total cycles since runnable init() 
	int cycleCounter;

	// Constructor
	DebugData() :v_desired(-1.0), v_ego(-1.0), dist_desired(-1.0), dist_ego_obj(-1.0), dist_ego_centerLane(0.0), cycleCounter(0), out() {}

};

//! Output structure
struct OutputData {

	/// output gas control 
	double out_gas = 0.0;

	/// output brake control 
	double out_brake = 0.0;

	/// output steering control 
	double out_steering = 0.0;

	/// debug info 
	DebugData debugOut;

	/// cycle counter for logging periodically 
	int cyclCntr = 0;

	// Constructor
	OutputData() :out_gas(0.0), out_brake(0.0), out_steering(0.0), debugOut(), cyclCntr(0) {}

	/** @brief print relevant output members to the log
	 *  @return void
	 */
	void print() {
		cyclCntr++;
		if (cyclCntr % 1000 == 0) {
			Log("Output Gas: %f\n", out_gas);
			Log("Output Brake: %f\n", out_brake);
			Log("Output Steering: %f\n", out_steering);
			Log("Debug Velocity Ego, Desired: %f, %f\n", debugOut.v_ego, debugOut.v_desired);
			Log("Debug Distance to Object, Desired: %f, %f\n", debugOut.dist_ego_obj, debugOut.dist_desired);
			Log("\n\n\n\n");
		}
	}

};