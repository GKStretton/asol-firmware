#include "controller.h"
#include "../middleware/sleep.h"
#include "../middleware/logger.h"

Status Controller::evaluateCalibration(State *s) {
	// Only auto-calibrate if last shutdown was safe.
	if (Sleep::GetLastSleepStatus() != Sleep::SAFE && !s->overrideCalibrationBlock) {
		return FAILURE;
	}

	// prevent autocalibration after manual clear
	if (s->calibrationCleared) {
		Logger::Warn("Calibration cleared, preventing auto-calibration");
		return FAILURE;
	}

	// Iterate steppers, moving such that they calibrate.
	bool anyRunning = false;
	int n = 5;
	UnitStepper* steppers[5] = {
		&(s->ringStepper),
		&(s->zStepper),
		&(s->yawStepper),
		&(s->pitchStepper),
		&(s->pipetteStepper),
	};
	for (int i = 0; i < n; i++) {
		if (steppers[i]->IsCalibrated()) {
			steppers[i]->setSpeed(0);
		} else {
			anyRunning = true;
			if (steppers[i]->HasLimitSwitchBeenPressed()) {
				steppers[i]->setSpeed(100); // release switch speed
			} else {
				steppers[i]->setSpeed(-300); // press switch speed
			}
		}
	}

	return anyRunning ? RUNNING : SUCCESS;
}