#include "controller.h"
#include "../app/navigation.h"
#include "../config.h"
#include "../middleware/logger.h"
#include "../middleware/sleep.h"
#include "../drivers/i2c_eeprom.h"
#include "state_report.h"

void Controller::autoUpdate(State *s) {
	// wake steppers
	digitalWrite(STEPPER_SLEEP, HIGH);

	// If shutting down
	if (s->shutdownRequested) {
		StateReport_SetStatus(machine_Status_SHUTTING_DOWN);
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
		StateReport_SetStatus(machine_Status_CALIBRATING);
		s->postCalibrationStopCalled = false;
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

	s->ringStepper.moveTo(s->ringStepper.UnitToPosition(s->target_ring));

	// No dye
	if (DO_DYE_COLLECTION && (s->pipetteState.spent || s->collectionInProgress)) {
		if (s->collectionRequest.requestCompleted) {
			StateReport_SetStatus(machine_Status_IDLE);
			Logger::Debug("No collection request, idling...");

			// Nothing to do. Wait at outer handover
			if (s->forceIdleLocation) s->SetGlobalNavigationTarget(IDLE_LOCATION);
			Navigation::UpdateNodeNavigation(s);
			return;
		} else {
			Logger::Debug("Collection in progress...");

			s->collectionInProgress = true;
			Status status = evaluatePipetteCollection(s);
			if (status == RUNNING || status == FAILURE) {
				StateReport_SetStatus(machine_Status_COLLECTING);
				return;
			}
			Logger::Debug("Collection complete");
			s->collectionInProgress = false;
			s->collectionRequest.requestCompleted = true;
		}
	}

	// At this point, we have collected liquid from a vial

	Navigation::SetGlobalNavigationTarget(s, INVERSE_KINEMATICS_POSITION);
	Status status = Navigation::UpdateNodeNavigation(s);
	// Block until we're in a safe dispense location
	if (status == RUNNING || status == FAILURE) {
		StateReport_SetStatus(machine_Status_NAVIGATING_IK);
		return;
	}

	// At this point, we have liquid from a vial and are in IK range

	status = evaluateIK(s);
	if (status == FAILURE) {
		Logger::Error("evaluate IK failed, returning");
		StateReport_SetStatus(machine_Status_ERROR);
		return;
	} else if (status == SUCCESS) {
		// Tip is stationary.
		// Fallthrough, allowing dispense
	} else if (status == RUNNING) {
		// block dispense if still moving
		StateReport_SetStatus(machine_Status_NAVIGATING_IK);
		return;
	}

	//todo: z control, depending on dispense too

	//? unify CollectionRequest.ulVolume and PipetteState.ulVolumeHeldTarget???
	// because you can just decrease the collection volume as it gets dispensed?
	// But will this work at edge cases?

	status = evaluatePipetteDispense(s);
	if (status == RUNNING) {
		StateReport_SetStatus(machine_Status_DISPENSING);
	}
	// Nothing to do, because once complete, the collection code takes over
	// on the next iteration
}