#pragma once

#include "Input.h"
#include "Output.h"
#include "LongController.h"
#include "LatController.h"


/** @file Highway_Pilot.h
 *  @brief Parent class that defines the functionality
 *  @author Oliver Altergott
 */

 //! Highway Pilot Parent Class
class Highway_Pilot {

public:
	// Constructor
	Highway_Pilot();

	/** @brief init method
	 *  @return void
	 */
	void init();

	/** @brief main public method for execution
	 *  @return void
	 */
	void run();

	/** @brief getter method for cycle counter
	 *  @return pointer to cycle counter
	*/
	int* getCycleCounterPtr();

	/** @brief getter method for lane change request
	 *  @return pointer to lane change request integer
	 */
	int* getLaneChangeReqPtr();

	/** @brief setter method for input struct
	 *   @return reference to the input
	 */
	InputData& setInput();

	/** @brief getter method for input struct
	 *   @return constant reference to the input
	 */
	const InputData& getInput();


	/** @brief setter method for output struct
	 *   @return reference to the output
	*/
	OutputData& setOutput();

	/** @brief getter method for output struct
	 *   @return constant reference to the output
	 */
	const OutputData& getOutput();



private:
	/// instance of input structure
	InputData input;

	/// instance of long controller
	LongitudinalController longController;
	/// instance of lat controller
	LatController latController;

	/// instance of output struct
	OutputData output;

	/// current set lane
	ELane currentLane;

	/// lane change timer for debounce
	int laneChangeTimer;

	/// user input lane change request: 0=stay, -1 = right lane, 1=left lane
	int laneChangeReq;
	int lastLaneChangeReq;

};