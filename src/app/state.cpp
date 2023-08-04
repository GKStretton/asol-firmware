#include "state.h"
#include "../calibration.h"
#include "../app/navigation.h"
#include "../app/state_report.h"
#include "../drivers/UnitStepper.h"
#include "../calibration.h"
#include "../config.h"

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
	this->postCalibrationHandlerCalled = false;
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
	this->overrideCalibrationBlock = false;
	this->rinseStatus = machine_RinseStatus_RINSE_COMPLETE;

	StateReport_SetMode(machine_Mode_UNDEFINED_MODE);
	StateReport_Update(this);
}

void State::SetGlobalNavigationTarget(machine_Node n) {
	//todo: refactor so all navigation state is inside navigation (make nav a
	//todo: class)
	Navigation::SetGlobalNavigationTarget(this, n);
}

State CreateStateObject() {
	return {
		updatesPerSecond: 0,
		lastNode: machine_Node_HOME,
		localTargetNode: machine_Node_UNDEFINED,
		globalTargetNode: machine_Node_HOME,
		manualRequested: false,
		lastControlUpdate: 0,
		lastDataUpdate: 0,
		pitchStepper: UnitStepper(PITCH_STEPPER_STEP, PITCH_STEPPER_DIR, 16, 0.44, -2.5, 90),
		yawStepper: UnitStepper(YAW_STEPPER_STEP, YAW_STEPPER_DIR, 8, 0.36, YAW_ZERO_OFFSET, 198),
		zStepper: UnitStepper(Z_STEPPER_STEP, Z_STEPPER_DIR, 4, 0.04078, 1, 73),
		ringStepper: UnitStepper(RING_STEPPER_STEP, RING_STEPPER_DIR, 32, 0.4, RING_ZERO_OFFSET, 195),
		pipetteStepper: UnitStepper(PIPETTE_STEPPER_STEP, PIPETTE_STEPPER_DIR, 2, 0.9, 0, 1000),
		target_x: 0.0,
		target_y: 0.0,
		target_ring: RING_ZERO_OFFSET,
		target_yaw: 0.0,
		collectionRequest: {true, 0, 0, 0.0},
		pipetteState: {true, 0, 0.0, false, 0},
		collectionInProgress: false,
		shutdownRequested: false,
		calibrationCleared: false,
		postCalibrationHandlerCalled: false,
		forceIdleLocation: true,
		fluidRequest: {FluidType::FLUID_UNDEFINED, false, 0, 0, true},
		ik_target_z: IK_Z,
		startup_counter: 0,
		overrideCalibrationBlock: false,
		rinseStatus: machine_RinseStatus_RINSE_COMPLETE
	};
}