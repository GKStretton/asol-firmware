#include "controller.h"
#include "../app/navigation.h"
#include "../config.h"
#include "../middleware/logger.h"

void Controller::autoUpdate(State *s) {
	// if not calibrated
	if (!s->IsArmCalibrated() || !s->ringStepper.IsCalibrated()) {
		manualUpdate(s);
		//todo: autocalibration
		return;
	}

	// wake steppers
	digitalWrite(STEPPER_SLEEP, HIGH);

	// No dye
	if (DO_DYE_COLLECTION && (s->GetPipetteVolumeHeld() <= 0 || s->collectionInProgress)) {
		if (s->collectionRequest.requestCompleted) {
			// Nothing to do. Wait at outer handover
			s->globalTargetNode = OUTER_HANDOVER;
			Navigation::UpdateNodeNavigation(s);
			return;
		} else {
			s->collectionInProgress = true;
			Status status = evaluatePipetteCollection(s);
			if (status == RUNNING || status == FAILURE) return;
			s->collectionInProgress = false;
		}
	}

	Navigation::UpdateNodeNavigation(s);
	//! Continue once all above is tested, and remove line above

	// Now we have dye

	// s->globalTargetNode = INVERSE_KINEMATICS_POSITION;
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