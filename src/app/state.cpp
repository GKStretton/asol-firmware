#include "state.h"

bool State::IsArmCalibrated() {
	return zStepper.IsCalibrated() && pitchStepper.IsCalibrated() && yawStepper.IsCalibrated();
}
