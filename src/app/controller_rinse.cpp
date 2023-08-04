#include "controller.h"
#include "navigation.h"
#include "../calibration.h"
#include "../middleware/logger.h"

Status Controller::evaluateRinse(State *s) {
	// Ensure valid mode	
	if (
		s->rinseStatus == machine_RinseStatus_RINSE_UNDEFINED ||
		s->rinseStatus == machine_RinseStatus_RINSE_COMPLETE
	) {
		Logger::Error("rinse status should not be " + String(s->rinseStatus) + " inside rinse controller");
		return FAILURE;
	}

	// Ensure buffer 
	if (s->pipetteState.spent) {
		// allow cancelling rinse if a collection is requested with same vial
		// as was held
		if (
			!s->collectionRequest.requestCompleted &&
			s->collectionRequest.vialNumber == s->pipetteState.vialHeld
		) {
			s->rinseStatus = machine_RinseStatus_RINSE_COMPLETE;
			return SUCCESS;
		}

		// Go to entry
		s->SetGlobalNavigationTarget(machine_Node_RINSE_CONTAINER_ENTRY);
		Status status = Navigation::UpdateNodeNavigation(s);
		if (status == RUNNING || status == FAILURE) return status;

		// Intake to buffer
		s->pipetteStepper.moveTo(s->pipetteStepper.UnitToPosition(PIPETTE_BUFFER));
		if (!s->pipetteStepper.AtTarget())
			return RUNNING;
		s->pipetteState.spent = false;
	}

	// intake water
	if (s->rinseStatus == machine_RinseStatus_RINSE_REQUESTED) {
		// Goto low position, in water
		s->SetGlobalNavigationTarget(machine_Node_RINSE_CONTAINER_LOW);
		Status status = Navigation::UpdateNodeNavigation(s);
		if (status == RUNNING || status == FAILURE) return status;

		s->pipetteStepper.moveTo(s->pipetteStepper.UnitToPosition(PIPETTE_BUFFER + PIPETTE_RINSE));
		if (!s->pipetteStepper.AtTarget())
			return RUNNING;
		s->rinseStatus = machine_RinseStatus_RINSE_EXPELLING;
	}
	
	// expel above water
	if (s->rinseStatus == machine_RinseStatus_RINSE_EXPELLING) {
		// Goto high position
		s->SetGlobalNavigationTarget(machine_Node_RINSE_CONTAINER_ENTRY);
		Status status = Navigation::UpdateNodeNavigation(s);
		if (status == RUNNING || status == FAILURE) return status;

		// pipette back to spent
		s->pipetteStepper.moveTo(s->pipetteStepper.UnitToPosition(0));
		if (!s->pipetteStepper.AtTarget())
			return RUNNING;
		s->pipetteState.spent = true;

		s->rinseStatus = machine_RinseStatus_RINSE_COMPLETE;
		return SUCCESS;
	}

	Logger::Error("Unexpected rinse controller logic flow");
	return FAILURE;
}