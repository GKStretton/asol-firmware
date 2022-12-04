#include "controller.h"
#include "../app/navigation.h"
#include "../config.h"
#include "../middleware/logger.h"
#include "../middleware/sleep.h"
#include "../drivers/i2c_eeprom.h"

void Controller::autoUpdate(State *s) {
	// wake steppers
	digitalWrite(STEPPER_SLEEP, HIGH);

	// If shutting down
	if (s->shutdownRequested) {
		Status status = evaluateShutdown(s);
		if (status == RUNNING) {
			return;
		} else if (status == FAILURE) {
			// safe shutdown flag already 0 as is this is set on wake
			Sleep::Sleep(Sleep::UNKNOWN);
			s->shutdownRequested = false;
			return;
		} else {
			// success, safe shutdown
			Sleep::Sleep(Sleep::SAFE);
			s->shutdownRequested = false;
			return;
		}
	}

	// if not calibrated
	if (!s->IsFullyCalibrated()) {
		Status status = evaluateCalibration(s);
		if (status == RUNNING) {
			return;
		} else if (status == FAILURE) {
			manualUpdate(s);
			return;
		}
		// if success, just continue
	}

	// set speed to 0 once after calibration so motors don't keep moving
	if (s->IsFullyCalibrated() && !s->postCalibrationStopCalled) {
		s->yawStepper.setSpeed(0);
		s->pitchStepper.setSpeed(0);
		s->zStepper.setSpeed(0);
		s->ringStepper.setSpeed(0);
		s->pipetteStepper.setSpeed(0);
		s->postCalibrationStopCalled = true;
		Logger::Debug("Set all motors to speed 0 after calibration");
	}

	// No dye
	if (DO_DYE_COLLECTION && (s->pipetteState.spent || s->collectionInProgress)) {
		if (s->collectionRequest.requestCompleted) {
			Logger::Debug("No collection request, idling...");
			// Nothing to do. Wait at outer handover
			s->SetGlobalNavigationTarget(IDLE_LOCATION);
			Navigation::UpdateNodeNavigation(s);
			return;
		} else {
			Logger::Debug("Collection in progress...");
			s->collectionInProgress = true;
			Status status = evaluatePipetteCollection(s);
			if (status == RUNNING || status == FAILURE) return;
			Logger::Debug("Collection complete");
			s->collectionInProgress = false;
			s->collectionRequest.requestCompleted = true;
		}
	} else {
		Logger::Debug("Skipping collection");
	}

	Navigation::UpdateNodeNavigation(s);
	evaluatePipetteDispense(s);

	//! Continue once all above is tested, and remove lines above

	// Now we have dye

	// s.Setblah(INVERSE_KINEMATICS_POSITION);
	// Status status = Navigation::UpdateNodeNavigation(s);
	// if (status == RUNNING || status == FAILURE) return;

	//! Now we have dye and are in IK range

	// evaluateIk()
	//todo: drop into IK land. control Z based on requested value in state
	//todo: and go to the specified xy
	//todo: but don't return if running, continue to dispense code

	//todo: s->pipetteStepper.moveTo(toUnits(s->pipetteState.ulVolumeHeldTarget))

	//? unify CollectionRequest.ulVolume and PipetteState.ulVolumeHeldTarget???
}