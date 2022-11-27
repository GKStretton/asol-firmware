#include "controller.h"
#include "../app/navigation.h"
#include "../config.h"

void Controller::autoUpdate(State *s) {
	// if not calibrated
	if (!s->IsArmCalibrated() || !s->ringStepper.IsCalibrated()) {
		manualUpdate(s);
		//todo: autocalibration
		return;
	}

	// wake steppers
	digitalWrite(STEPPER_SLEEP, HIGH);

	Status status = Navigation::UpdateNodeNavigation(s);
	if (status == RUNNING || status == FAILURE) return;

	//! Now guaranteed to be at global target navigation node


}