#pragma once

#include<Log.h>

/** @file Input.h
 *  @brief Defined parameters and input relevant data structure definition
 *  @author Oliver Altergott
 */


 //! Parameters

/// Maximum lateral acceleration for velocity setpoint 
static constexpr double ACC_Y_MAX = 1.5;

/// Velocity diff to object for obstacle classification (30km/h)
static constexpr double OBST_VEL_DIFF_TO_SETPOINT_THRESHOLD = -30.0 / 3.6;

/// Lookahead distance for egoLane obstacle detection
static constexpr double EGOLANE_OBST_LOOKAHEAD_DIST = 50.0;

/// Lookahead distance for neighborLane obstacle detection
static constexpr double NEIGHBORLANE_FREE_LOOKAHEAD_DIST = 60.0;

/// Debounce time for lane change (prevent lane changes during ongoing one)
static constexpr int LANE_CHANGE_TIME_DEBOUNCE = 6000;

/// Lane enums
enum class ELane {
	MIDDLE_LANE = 0,
	LEFT_LANE = 1,
	RIGHT_LANE = -1
};

//! Relevant Object Data Structure 
struct RelevantObject {
	int id;
	double rel_dist[2];
	double rel_vel;
	double abs_vel;

	/// Constructor
	RelevantObject() :id(-1), rel_dist{ 0.0,0.0 }, rel_vel(0.0), abs_vel(0.0){}

};

//! Lane Data Structure 
struct Lane {
	bool exists;
	double dev_dist;
	double dev_angl;

	// Constructor
	Lane() :exists(false), dev_dist(0.0), dev_angl(0.0) {}
};

//! Input Data Structure
struct InputData {

	/// counter for logging periodically 
	int cyclCntr;

	/// absolute vehicle velocity 
	double veh_vel_abs; 

	/// vehicle acceleration in x dir 
	double veh_accX;

	/// vehicle acceleration in y dir  
	double veh_accY;

	/// road curvature 
	double road_curvature;

	/// current speed limit on route
	double road_speed_limit;

	/// relevant object on egoLane 
	RelevantObject relObj;
	/// relevant object on left lane 
	RelevantObject leftRelObj;
	/// relevant object on right lane 
	RelevantObject rightRelObj;

	/// egoLane member 
	Lane egoLane;
	/// left lane member 
	Lane leftLane;
	/// right lane member 
	Lane rightLane;

	// Constructor
	InputData() :cyclCntr(0), veh_vel_abs(0.0), veh_accX(0.0), veh_accY(0.0), road_curvature(0.0), road_speed_limit(0.0), relObj(), leftRelObj(), rightRelObj(), egoLane(), leftLane(), rightLane() {}

	/** @brief setter for ego velocity
	 *  @param ego velocity (double)
	 *  @return void
	 */
	void setEgoVel(double f_egoVel) { veh_vel_abs = f_egoVel; }

	/** @brief setter for ego acceleration
	 *  @param acceleration in x (double)
	 *  @param acceleration in y (double)
	 *  @return void
	 */
	void setEgoAcc(double f_accX, double f_accY) {
		veh_accX = f_accX;
		veh_accY = f_accY;
	}


	/** @brief setter for speed limit
	 *  @param speed limit in m/s (double)
	 *  @return void
	 */
	void setSpeedLimit(double f_speedLimit) {
		road_speed_limit = f_speedLimit;
	}

	/** @brief setter for speed limit
	 *  @param road curvature in 1/m (double)
	 *  @return void
	 */
	void setRoadCurvature(double f_curvature) { road_curvature = f_curvature; }


	/** @brief resets all members
	 *  @return void
	 */
	void reset() {
		veh_vel_abs = 0.0;
		veh_accX = 0.0;
		veh_accY = 0.0;
		road_curvature = 0.0;
		road_speed_limit = 0.0;
		RelevantObject defaultObj;
		relObj = defaultObj;
		leftRelObj = defaultObj;
		rightRelObj = defaultObj;
		rightRelObj = defaultObj;
		Lane defaultLane;
		egoLane = defaultLane;
		leftLane = defaultLane;
		rightLane = defaultLane;
	}

	/** @brief periodically prints relevant members to the log
	 *  @return void
	 */
	void print() {
		cyclCntr++;
		if (cyclCntr % 1000 == 0) {
			Log("Input Vel: %f\n", veh_vel_abs);
			Log("Input ACC X,Y: %f, %f\n", veh_accX, veh_accY);
			Log("Input Road Speed Limit, Curvature: %f, %f\n", road_speed_limit, road_curvature);
			Log("Input EGO Lane Dev Dist, Angl: %f, %f\n", egoLane.dev_dist, egoLane.dev_angl);
			Log("Input LEFT Lane Exists, Dist: %d, %f\n", leftLane.exists, leftLane.dev_dist);
			Log("Input RIGHT Lane Exists, Dist: %d, %f\n", rightLane.exists, rightLane.dev_dist);
			Log("Input RelObject: ID, RelDist, RelVel: %d, %f, %f \n", relObj.id, relObj.rel_dist[0], relObj.rel_vel);
			Log("Input LeftRelObject: ID, RelDist, RelVel: %d, %f, %f \n", leftRelObj.id, leftRelObj.rel_dist[0], leftRelObj.rel_vel);
			Log("Input RightRelObject: ID, RelDist, RelVel: %d, %f, %f \n", rightRelObj.id, rightRelObj.rel_dist[0], rightRelObj.rel_vel);

			Log("\n\n");
		}
	}

};