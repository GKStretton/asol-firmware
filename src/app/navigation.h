#pragma once
#include "../app/state.h"
#include "../app/status.h"
#include "../app/node.h"
#include "../extras/machinepb/machine.pb.h"

namespace Navigation {
	Status UpdateNodeNavigation(State *s);
	// This changes global target and has safeties for if we change while in motion.
	void SetGlobalNavigationTarget(State *s, machine_Node n);
};