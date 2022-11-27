#include "controller.h"
#include "../app/navigation.h"
#include "../config.h"

void Controller::autoUpdate(State *s) {
	// wake steppers
	digitalWrite(STEPPER_SLEEP, HIGH);

	// if not calibrated

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