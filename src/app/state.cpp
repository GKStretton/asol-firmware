#include "state.h"
#include "../calibration.h"
#include "../app/navigation.h"
#include "../app/state_report.h"

bool State::IsArmCalibrated() {
	return
		zStepper.IsCalibrated() &&
		pitchStepper.IsCalibrated() &&
		yawStepper.IsCalibrated();
}

bool State::IsFullyCalibrated() {
	return
		zStepper.IsCalibrated() &&
		pitchStepper.IsCalibrated() &&
		yawStepper.IsCalibrated() &&
		ringStepper.IsCalibrated() &&
		pipetteStepper.IsCalibrated();
}

float State::GetPipetteVolumeHeld() {
	return pipetteStepper.PositionToUnit(pipetteStepper.currentPosition()) - PIPETTE_BUFFER;
}

void State::ClearState() {
	this->lastNode = machine_Node_HOME;
	this->localTargetNode = machine_Node_UNDEFINED;
	this->globalTargetNode = machine_Node_HOME;
	this->target_x = 0.0;
	this->target_y = 0.0;
	this->target_ring = RING_ZERO_OFFSET;
	this->target_yaw = 0.0;
	this->collectionRequest = {true, 0, 0, 0.0};
	this->pipetteState = {true, 0, 0.0, false, 0};
	this->collectionInProgress = false;
	this->shutdownRequested = false;
	this->calibrationCleared = false;
	this->postCalibrationStopCalled = false;
	this->manualRequested = false;
	this->forceIdleLocation = true;

	// clear calibration
	this->pitchStepper.MarkAsNotCalibrated();
	this->yawStepper.MarkAsNotCalibrated();
	this->zStepper.MarkAsNotCalibrated();
	this->ringStepper.MarkAsNotCalibrated();
	this->pipetteStepper.MarkAsNotCalibrated();

	this->ik_target_z = IK_Z;

	this->fluidRequest = {FLUID_UNDEFINED, false, 0, 0, true};

	StateReport_SetMode(machine_Mode_UNDEFINED_MODE);
	StateReport_Update(this);
}

void State::SetGlobalNavigationTarget(machine_Node n) {
	//todo: refactor so all navigation state is inside navigation (make nav a
	//todo: class)
	Navigation::SetGlobalNavigationTarget(this, n);
}