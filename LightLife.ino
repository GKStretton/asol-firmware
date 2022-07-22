#include <defs.h>
#include <simpleRPC.h>
#include "switch.h"
#include "stage.h"
#include "fluid.h"
#include "config.h"
#include "util.h"

void setup() {
	Serial.begin(9600);

	SetupSwitches();
	SetupStage();
	SetupFluid();
}


void loop() {
	interface(
		Serial,

		SetCameraState, "SetCameraState turns the DSLR on or off. @on: state",
		SetRingLightState, "SetRingLightState turns the ring light on or off. @on: state",
		SetMainsState, "SetMainsState switches the AC power input. @on: state",
		SetLED, "SetLED: Turn the on-board led on or off. @on: boolean on or off.",

		Drain, "Drain: Set drainage state. @drain: boolean for whether to be draining",

		SetAirFlow,   "SetAirFlow: Turn the flow of air or off in the tubing @flow: boolean for flow",
		SetFlushFlow,  "SetFlushFlow: Turn the dispension of the flushing fluid on or off @flow: boolean for flow",
		SetCanvasFlow, "SetCanvasFlow: Turn the dispension of the canvas fluid on or off @flow: boolean for flow"
	);
}