#include "controller.h"
#include "../app/navigation.h"

Status Controller::evaluateShutdown(State *s) {
	// this var helps us do parallel movements within this function
	bool somethingRunning = false;
	bool failureMarked = false;

	// Ring shutdown behaviour, go to min
	if (s->ringStepper.IsCalibrated()) {
		s->ringStepper.moveTo(s->ringStepper.UnitToPosition(
			s->ringStepper.GetMinUnit()
		));
		if (!s->ringStepper.AtTarget())
			somethingRunning = true;
	}

	// Arm shutdown behaviour
	if (s->IsArmCalibrated()) {
		s->SetGlobalNavigationTarget(Node::HOME);
		Status status = Navigation::UpdateNodeNavigation(s);
		if (status == FAILURE) failureMarked = true;
		else if (status == RUNNING) somethingRunning = true;
		// if succ that's good, defualt
	} else {
		failureMarked = true;
	}

	if (somethingRunning)
		return RUNNING;
	else if (failureMarked)
		return FAILURE;
	else 
		return SUCCESS;
}