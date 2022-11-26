#include "controller.h"
#include "../app/navigation.h"
#include "../config.h"

void Controller::autoUpdate(State *s) {
	// wake steppers
	digitalWrite(STEPPER_SLEEP, HIGH);

	Navigation::UpdateNodeNavigation(s);
}