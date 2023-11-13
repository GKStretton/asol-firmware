#include "controller.h"
#include "../app/navigation.h"
#include "../config.h"
#include "../middleware/logger.h"
#include "../middleware/sleep.h"
#include "../drivers/i2c_eeprom.h"
#include "../drivers/cover_servo.h"
#include "state_report.h"

// this structure is like a pseudo behavior tree. Every update, the program
// steps through here to decide what to do next. sub-functions return 
// RUNNING / SUCCESS / FAILURE to indicate what happened. Higher priority
// tasks are at the start of the function, often returning early if they
// are running.
void Controller::autoUpdate(State *s) {
	// wake steppers
	digitalWrite(STEPPER_SLEEP, HIGH);

	// If shutting down
	if (s->shutdownRequested) {
		// including autonomous declaration in multiple places because
		// if mode changes from manual to auto to manual, a state report will be sent
		StateReport_SetMode(machine_Mode_AUTONOMOUS);
		StateReport_SetStatus(machine_Status_SHUTTING_DOWN);
		Status status = evaluateShutdown(s);
		if (status == RUNNING) {
			return;
		} else if (status == FAILURE) {
			// safe shutdown flag already 0 as this is set on wake
			Sleep::Sleep(Sleep::UNKNOWN);
			s->shutdownRequested = false;
			return;
		} else {
			CoverServo_Close();
			// success, safe shutdown
			Sleep::Sleep(Sleep::SAFE);
			s->shutdownRequested = false;
			return;
		}
	}

	// if not calibrated
	if (!s->IsFullyCalibrated()) {
		StateReport_SetStatus(machine_Status_CALIBRATING);
		s->postCalibrationHandlerCalled = false;
		Status status = evaluateCalibration(s);
		if (status == RUNNING) {
			StateReport_SetMode(machine_Mode_AUTONOMOUS);
			return;
		} else if (status == FAILURE) {
			manualUpdate(s, true);
			return;
		}
		// if success, just continue (success never called though)
	}
	StateReport_SetMode(machine_Mode_AUTONOMOUS);

	// set speed to 0 once after calibration so motors don't keep moving
	if (s->IsFullyCalibrated() && !s->postCalibrationHandlerCalled) {
		s->yawStepper.setSpeed(0);
		s->pitchStepper.setSpeed(0);
		s->zStepper.setSpeed(0);
		s->ringStepper.setSpeed(0);
		s->pipetteStepper.setSpeed(0);
		s->postCalibrationHandlerCalled = true;
		Logger::Debug("Set all motors to speed 0 after calibration");

		if (Sleep::GetLastSleepStatus() == Sleep::SAFE) {
			CoverServo_Open();
		}
	}

	s->ringStepper.moveTo(s->ringStepper.UnitToPosition(s->target_ring));

	if (s->rinseStatus != machine_RinseStatus_RINSE_COMPLETE) {
		Logger::Debug("Rinse in progress...");
		StateReport_SetStatus(machine_Status_RINSING_PIPETTE);

		Status status = evaluateRinse(s);
		if (status != SUCCESS) {
			return;
		}
	}

	// No dye
	if (DO_DYE_COLLECTION && (s->pipetteState.spent || s->collectionInProgress)) {
		if (s->collectionRequest.requestCompleted) {
			Logger::Debug("No collection request, idling...");

			// Nothing to do. Wait at outer handover
			if (s->forceIdleLocation) s->SetGlobalNavigationTarget(machine_Node_IDLE_LOCATION);
			Status status = Navigation::UpdateNodeNavigation(s);
			if (status == RUNNING || !s->ringStepper.AtTarget()) {
				StateReport_SetStatus(machine_Status_IDLE_MOVING);
			} else {
				StateReport_SetStatus(machine_Status_IDLE_STATIONARY);
			}
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

	Navigation::SetGlobalNavigationTarget(s, machine_Node_INVERSE_KINEMATICS_POSITION);
	Status status = Navigation::UpdateNodeNavigation(s);
	// Block until we're in a safe dispense location
	if (status == RUNNING || status == FAILURE) {
		StateReport_SetStatus(machine_Status_NAVIGATING_OUTER);
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

	status = evaluatePipetteDispense(s);
	if (status == RUNNING) {
		StateReport_SetStatus(machine_Status_DISPENSING);
		return;
	}

	StateReport_SetStatus(machine_Status_WAITING_FOR_DISPENSE);

	// Nothing else to do, because once complete, the collection code takes over
	// on the next iteration
}