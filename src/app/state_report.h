#pragma once

#include "state.h"
#include "../protos/machinepb/machine.pb.h"

// Called to check for updates and publish a report if anything's changed.
// Call every control update.
void StateReport_Update(State *s);

/*
PUBLIC SETTERS FOR MANUAL UPDATE OF STATE REPORT
*/

void StateReport_SetMode(machine_Mode mode);
void StateReport_SetStatus(machine_Status status);