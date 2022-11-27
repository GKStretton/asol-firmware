#include "state.h"
#include "../calibration.h"

bool State::IsArmCalibrated() {
	return zStepper.IsCalibrated() && pitchStepper.IsCalibrated() && yawStepper.IsCalibrated();
}

float State::GetPipetteVolumeHeld() {
	return pipetteStepper.PositionToUnit(pipetteStepper.currentPosition()) - PIPETTE_BUFFER;
}