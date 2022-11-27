#include "controller.h"
#include "../app/navigation.h"
#include "../config.h"

void Controller::autoUpdate(State *s) {
	// if not calibrated
	if (!s->IsArmCalibrated()) {
		manualUpdate(s);
		//todo: autocalibration
		return;
	}

	// wake steppers
	digitalWrite(STEPPER_SLEEP, HIGH);

	Status status = Navigation::UpdateNodeNavigation(s);
	if (status == RUNNING) {
		return;
	}
	if (status == FAILURE) {
		return;
	}
	if (status == SUCCESS) {

	}
}