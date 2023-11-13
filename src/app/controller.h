#pragma once
#include "../app/state.h"
#include "../app/status.h"

class Controller {
public:
	void Init(State *s);
	void Update(State *s);
	void NewFluidRequest(State *s, FluidType fluidType, float volume_ml, bool open_drain);
private:
	void fluidInit(State *s);

	void autoUpdate(State *s);
	void manualUpdate(State *s, bool calibrating);
	void fluidUpdate(State *s);
	// Behaviour "node" for fluid collection, without assumptions
	Status evaluatePipetteCollection(State *s);
	Status evaluatePipetteDispense(State *s);
	Status evaluateIK(State *s);
	Status evaluateShutdown(State *s);
	Status evaluateCalibration(State *s);
	Status evaluateRinse(State *s);
};