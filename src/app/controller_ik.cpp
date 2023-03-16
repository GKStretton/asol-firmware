#include "controller.h"
#include "../app/navigation.h"
#include "../middleware/logger.h"
#include "../calibration.h"
#include "../common/ik_algorithm.h"

Status Controller::evaluateIK(State *s) {
	if (
		s->pitchStepper.PositionToUnit(s->pitchStepper.currentPosition()) < CENTRE_PITCH - 1 ||
		s->zStepper.PositionToUnit(s->zStepper.currentPosition()) < MIN_BOWL_Z ||
		s->yawStepper.PositionToUnit(s->yawStepper.currentPosition()) > MAX_BOWL_YAW
	)
	{
		Logger::Error("Invalid position for IK");
		return FAILURE;
	}

	s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(CENTRE_PITCH));

	// note, ring is pre-emptively set elsewhere too, so this line is just ensurance
	s->ringStepper.moveTo(s->ringStepper.UnitToPosition(s->target_ring));

	//? idea: s->yawStepper.setSpeed(s->ringStepper.distanceToGo()) 
	//? to time the yaw and ring to arrive at the same time?
	s->yawStepper.moveTo(s->yawStepper.UnitToPosition(s->target_yaw));

	s->zStepper.moveTo(s->zStepper.UnitToPosition(s->ik_target_z));

	if (s->pitchStepper.AtTarget() && s->ringStepper.AtTarget() && s->yawStepper.AtTarget() && s->zStepper.AtTarget()) return SUCCESS;

	return RUNNING;
}