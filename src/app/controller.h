#pragma once
#include "../app/state.h"
#include "../app/status.h"

class Controller {
public:
	void Update(State *s);
private:
	void autoUpdate(State *s);
	void manualUpdate(State *s);
	// Behaviour "node" for fluid collection, without assumptions
	Status evaluatePipetteCollection(State *s);
	Status evaluatePipetteDispense(State *s);
	Status evaluateIK(State *s);
};