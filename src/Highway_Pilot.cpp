#include "Highway_Pilot.h"


Highway_Pilot::Highway_Pilot() : input(), output(), longController(), currentLane(ELane::MIDDLE_LANE), laneChangeTimer(0), laneChangeReq(0), lastLaneChangeReq(0) {}

void Highway_Pilot::init() {
	output.debugOut.cycleCounter = 0;
}

void Highway_Pilot::run() {


	/********LONGITUDINAL*LOGIC*********/

	const auto relevantDist = input.veh_vel_abs * 4.0 + 10.0;
	const auto relevantObjAvailable = (input.relObj.id > 0 && input.relObj.rel_dist[0] < relevantDist);

	if (relevantObjAvailable) {
		longController.control_dist(input, output);
	}
	else {
		longController.control_vel(input, output);
	}


	/***********LATERAL*LOGIC**********/

	// Keep desired lane
	latController.control_lane_deviation(input, output, currentLane);



	// if obstacle in ego lane & left lane available & free -> change to left lane
	// else follow object or free drive 

	const bool obstacle_egoLane_available = ((input.relObj.id > 0) && (input.relObj.abs_vel - output.debugOut.v_desired) < OBST_VEL_DIFF_TO_SETPOINT_THRESHOLD) && (input.relObj.rel_dist[0] < EGOLANE_OBST_LOOKAHEAD_DIST);

	const bool object_relativeLeftLane_near = (input.leftRelObj.id > 0) && (input.leftRelObj.rel_dist[0] < NEIGHBORLANE_FREE_LOOKAHEAD_DIST);
	const bool object_relativeRightLane_near = (input.rightRelObj.id > 0) && (input.rightRelObj.rel_dist[0] < NEIGHBORLANE_FREE_LOOKAHEAD_DIST);

	const bool absLeftLane_available_free = !object_relativeLeftLane_near && input.leftLane.exists;
	const bool absRightLane_available_free = !object_relativeRightLane_near && input.rightLane.exists;



	if (obstacle_egoLane_available) {

		if (absLeftLane_available_free && laneChangeTimer == 0) {
			laneChangeTimer++;
			currentLane = ELane::LEFT_LANE;
		}
		else {
			// following handled by long controller independently
		}
	}

	// if we are on the relative left lane, and the relative right is free now, change lane to the right
	// assume the original starting lane, i.e. absolute middle is always available
	if (currentLane == ELane::LEFT_LANE && !object_relativeRightLane_near && laneChangeTimer == 0) {
		currentLane = ELane::MIDDLE_LANE;
	}

	// User requested lane change
	if ((laneChangeReq == 0) && (laneChangeReq != lastLaneChangeReq)) {

		if (currentLane == ELane::LEFT_LANE && !object_relativeRightLane_near && laneChangeTimer == 0) {
			currentLane = ELane::MIDDLE_LANE;
			laneChangeTimer++;
		}
		else if (currentLane == ELane::RIGHT_LANE && !object_relativeLeftLane_near && laneChangeTimer == 0) {
			currentLane = ELane::MIDDLE_LANE;
			laneChangeTimer++;
		}

	}
	else if ((laneChangeReq == 1) && (laneChangeReq != lastLaneChangeReq) && absLeftLane_available_free && laneChangeTimer == 0) {
		currentLane = ELane::LEFT_LANE;
		laneChangeTimer++;
	}
	else if ((laneChangeReq == -1) && (laneChangeReq != lastLaneChangeReq) && absRightLane_available_free && laneChangeTimer == 0) {
		currentLane = ELane::RIGHT_LANE;
		laneChangeTimer++;
	}

	if (laneChangeTimer > 0) {
		laneChangeTimer++;
		if (laneChangeTimer > LANE_CHANGE_TIME_DEBOUNCE) {
			laneChangeTimer = 0;
		}
	}

	lastLaneChangeReq = laneChangeReq;

	output.debugOut.v_ego = input.veh_vel_abs;
	output.debugOut.dist_ego_obj = input.relObj.rel_dist[0];
	output.debugOut.cycleCounter++;

}

int* Highway_Pilot::getCycleCounterPtr() {
	return &output.debugOut.cycleCounter;
}

int* Highway_Pilot::getLaneChangeReqPtr() { return &laneChangeReq; }


InputData& Highway_Pilot::setInput() {
	return input;
}

const InputData& Highway_Pilot::getInput() {
	return input;
}

OutputData& Highway_Pilot::setOutput() {
	return output;
}

const OutputData& Highway_Pilot::getOutput() {
	return output;
}