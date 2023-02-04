#include "controller.h"

void Controller::fluidInit(State *s) {
	//todo: load restore state
}

void Controller::fluidUpdate(State *s) {
	if (s->fluidState.complete) {
		return;
	}
	// we have an unfilled state change

	if (s->fluidState.startTime == 0) {
		s->fluidState.startTime = millis();
	}

	// todo: work out how long the operation takes
	// todo: if this time has not elapsed, set in_progress to true
	bool in_progress = false;

	// idea: instead of below, calculate pin_num and time from the request?
	if (s->fluidState.fluidType == FluidType::NONE) {
		// draining
	} else {
		// filling
	}

	// todo: remember to open the airflow once fluid dispense is complete
	// todo: this can just be based off the time as well.
}