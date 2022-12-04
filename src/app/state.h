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
};

struct State {
	bool IsArmCalibrated();
	float GetPipetteVolumeHeld();
	// Clears state to begin like new
	void ClearState();
	void SetGlobalNavigationTarget(Node n);

	int updatesPerSecond;
	// The most recent node to have been visited
	Node lastNode;
	// Most recent local node being navigated to
	Node localTargetNode;
	// The final goal node in a potentially multi-hop movement
	Node globalTargetNode;
	// If true, respect the fs-i6 controller
	bool manual;
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

	CollectionRequest collectionRequest;
	PipetteState pipetteState;

	// True if fluid is currently being taken up
	bool collectionInProgress;
	// True if we've received shutdown request that's not done yet
	bool shutdownRequested;
};