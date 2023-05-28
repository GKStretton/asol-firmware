#include "controller.h"
#include "../app/navigation.h"
#include "../calibration.h"
#include "../middleware/logger.h"

Status Controller::evaluateShutdown(State *s) {
	// this var helps us do parallel movements within this function
	bool somethingRunning = false;
	bool failureMarked = false;

	// Ring shutdown behaviour, go to min
	if (s->ringStepper.IsCalibrated()) {
		s->ringStepper.moveTo(s->ringStepper.UnitToPosition(
			s->ringStepper.GetMinUnit()
		));
		if (!s->ringStepper.AtTarget()) {
			somethingRunning = true;
			// go to ring 0 before arm action, to prevent cable hitting limit switch
			return RUNNING;
		}
	}

	// sometimes this hangs because the pipette stepper says it's
	// at -7 steps or so. Not sure why it doesn't just go to 0 and complete...
	// upon pressing the limit switch, everything shuts down.
	// disabling for now
	// if (s->pipetteStepper.IsCalibrated()) {
	// 	s->pipetteStepper.moveTo(s->pipetteStepper.UnitToPosition(s->pipetteStepper.GetMinUnit()));
	// 	if (!s->pipetteStepper.AtTarget())
	// 		somethingRunning = true;
	// }

	// Arm shutdown behaviour
	if (s->IsArmCalibrated()) {
		s->SetGlobalNavigationTarget(machine_Node_HOME);
		Status status = Navigation::UpdateNodeNavigation(s);
		if (status == FAILURE) failureMarked = true;
		else if (status == RUNNING) somethingRunning = true;
		// if succ that's good, default
	} else {
		Logger::Warn("shutdown failure due to arm not calibrated");
		Logger::Warn("z calibrated: " + String(s->zStepper.IsCalibrated()));
		Logger::Warn("pitch calibrated: " + String(s->pitchStepper.IsCalibrated()));
		Logger::Warn("yaw calibrated: " + String(s->yawStepper.IsCalibrated()));
		failureMarked = true;
	}

	if (somethingRunning)
		return RUNNING;
	else if (failureMarked)
		return FAILURE;
	else 
		return SUCCESS;
}