#pragma once

#include "../app/node.h"
#include "../drivers/UnitStepper.h"

struct CollectionRequest {
	bool requestCompleted;
	int requestNumber;
	int vialNumber;
	float ulVolume;
};

struct PipetteState {
	// spent is true if we've dispensed past the buffer, if all is gone.
	bool spent;
	int vialHeld;
	float ulVolumeHeldTarget;
	// true once a dispense has been requested. Used to enable backlash
	// compensation on the first dispense so that the state report doesn't show
	// dispensing when waiting for first dispense.
	bool dispenseRequested;
	int dispenseRequestNumber;
};

enum FluidType {
	FLUID_UNDEFINED = 0,
	DRAIN = 1,
	WATER = 2,
	MILK = 3,
};

struct FluidRequest {
	FluidType fluidType;
	bool open_drain;
	float volume_ml;
	unsigned long startTime;
	bool complete;
};


struct State {
	// returns true if pitch, yaw and z are calibrated
	bool IsArmCalibrated();
	// returns true if pitch, yaw, z, ring, and pipette are calibrated
	bool IsFullyCalibrated();
	float GetPipetteVolumeHeld();
	// Clears state to begin like new
	void ClearState();
	void SetGlobalNavigationTarget(machine_Node n);

	// stepper ticks
	int updatesPerSecond;
	// The most recent node to have been visited
	machine_Node lastNode;
	// Most recent local node being navigated to
	machine_Node localTargetNode;
	// The final goal node in a potentially multi-hop movement
	machine_Node globalTargetNode;
	// If true, respect the fs-i6 controller
	bool manualRequested;
	// Timing
	unsigned long lastControlUpdate;
	unsigned long lastDataUpdate;

	// Steppers
	UnitStepper pitchStepper;
	UnitStepper yawStepper;
	UnitStepper zStepper;
	UnitStepper ringStepper;
	UnitStepper pipetteStepper;

	float target_x;
	float target_y;
	float target_ring;
	float target_yaw;

	CollectionRequest collectionRequest;
	PipetteState pipetteState;

	// True if fluid is currently being taken up
	bool collectionInProgress;
	// True if we've received shutdown request that's not done yet
	bool shutdownRequested;
	bool calibrationCleared;
	bool postCalibrationHandlerCalled;
	// if true, always go to the idle node when idle
	bool forceIdleLocation;

	FluidRequest fluidRequest;

	float ik_target_z;

	uint8_t startup_counter;

	bool overrideCalibrationBlock;

	machine_RinseStatus rinseStatus;
};

State CreateStateObject();