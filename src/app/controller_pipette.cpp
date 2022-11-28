#include "controller.h"
#include "../app/navigation.h"
#include "../calibration.h"

Status Controller::evaluatePipetteCollection(State *s) {
	// We have a request, time to collect dye!
	Node n = VialNumberToInsideNode(s->collectionRequest.vialNumber);
	s->globalTargetNode = n;
	Status status = Navigation::UpdateNodeNavigation(s);
	if (status == RUNNING || status == FAILURE) return;

	//! At inner node.

	// "reload" pipette (go to buffer point)
	if (s->pipetteState.spent) {
		s->pipetteStepper.moveTo(s->pipetteStepper.UnitToPosition(PIPETTE_BUFFER));
		if (!s->pipetteStepper.AtTarget())
			return RUNNING;
		s->pipetteState.spent = false;
	}

	s->zStepper.moveTo(s->zStepper.UnitToPosition(PIPETTE_INTAKE_Z));
	if (!s->zStepper.AtTarget()) {
		return RUNNING;

	s->pipetteStepper.moveTo(s->pipetteStepper.UnitToPosition(PIPETTE_BUFFER + s->collectionRequest.ulVolume));
	if (!s->pipetteStepper.AtTarget())
		return RUNNING;

	return SUCCESS;
}
