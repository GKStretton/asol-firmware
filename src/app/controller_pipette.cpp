#include "controller.h"
#include "../app/navigation.h"
#include "../calibration.h"
#include "../middleware/logger.h"

Status Controller::evaluatePipetteCollection(State *s) {
	// We have a request, time to collect dye!
	machine_Node n = VialNumberToInsideNode(s->collectionRequest.vialNumber);
	s->SetGlobalNavigationTarget(n);
	Status status = Navigation::UpdateNodeNavigation(s);
	if (status == RUNNING || status == FAILURE) return status;

	//! At inner node.
	Logger::Debug("evaluatePipetteCollection at correct inner node, doing collection...");

	// "reload" pipette (go to buffer point)
	if (s->pipetteState.spent) {
		s->pipetteStepper.moveTo(s->pipetteStepper.UnitToPosition(PIPETTE_BUFFER));
		if (!s->pipetteStepper.AtTarget())
			return RUNNING;
		s->pipetteState.spent = false;
	}

	s->zStepper.moveTo(s->zStepper.UnitToPosition(PIPETTE_INTAKE_Z));
	if (!s->zStepper.AtTarget())
		return RUNNING;

	s->pipetteStepper.moveTo(s->pipetteStepper.UnitToPosition(PIPETTE_BUFFER + s->collectionRequest.ulVolume));
	if (!s->pipetteStepper.AtTarget())
		return RUNNING;
	
	s->pipetteState.ulVolumeHeldTarget = s->collectionRequest.ulVolume;
	s->pipetteState.vialHeld = s->collectionRequest.vialNumber;
	s->pipetteState.dispenseRequested = false;

	return SUCCESS;
}

Status Controller::evaluatePipetteDispense(State *s) {
	float target;
	if (s->pipetteState.ulVolumeHeldTarget <= 0) {
		target = 0;
	} else {
		target = PIPETTE_BUFFER + s->pipetteState.ulVolumeHeldTarget - 
			(s->pipetteState.dispenseRequested ? PIPETTE_BACKLASH_UL : 0);
	}
	
	s->pipetteStepper.moveTo(s->pipetteStepper.UnitToPosition(target));
	if (!s->pipetteStepper.AtTarget()) {
		return RUNNING;
	}
	
	// mark as spent if at target of 0
	if (s->pipetteState.ulVolumeHeldTarget <= 0 && !s->pipetteState.spent) {
		s->pipetteState.spent = true;
		// trigger rinse after final dispense
		s->rinseStatus = machine_RinseStatus_RINSE_REQUESTED;
		// Running here so that we don't get a single report of WAITING_FOR_DISPENSE
		// after it's spent
		return RUNNING;
	}

	// Success if dispensed and not spent
	return SUCCESS;
}
