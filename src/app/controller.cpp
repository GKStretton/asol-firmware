#include "controller.h"
#include "../drivers/fs-i6.h"
#include "../config.h"
#include "state_report.h"

unsigned long lastControlUpdate = millis();

void Controller::Init(State *s) {
	fluidInit(s);
}

void Controller::Update(State *s) {
	if (millis() - lastControlUpdate > 100)
	{
		lastControlUpdate = millis();

		bool boardSwitchA = digitalRead(SWITCH_A);
		if (boardSwitchA || s->manualRequested) {
			manualUpdate(s, false);
		} else {
			autoUpdate(s);
		}

		// Can happen regardless of mode
		fluidUpdate(s);

		StateReport_Update(s);
	}
}