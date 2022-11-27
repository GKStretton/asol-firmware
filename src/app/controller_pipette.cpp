#include "controller.h"
#include "../app/navigation.h"

Status evaluatePipetteCollection(State *s) {
	// We have a request, time to collect dye!
	Node n = VialNumberToInsideNode(s->collectionRequest.vialNumber);
	Status status = Navigation::UpdateNodeNavigation(s);
	if (status == RUNNING || status == FAILURE) return;

	//! At inner node.

	//todo: goto pipette buffer position
	//todo: go down to liquid level
	//todo: draw up s->collectionRequest.ulVolume
	//todo: (optional) go up out of liquid.
	//todo: return SUCCESS
}
