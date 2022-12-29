#include "controller.h"
#include "../drivers/fs-i6.h"
#include "../config.h"
#include "state_report.h"

unsigned long lastControlUpdate = millis();

void Controller::Update(State *s) {
	if (millis() - lastControlUpdate > 100)
	{
		lastControlUpdate = millis();

		bool boardSwitchA = digitalRead(SWITCH_A);
		if (boardSwitchA || s->manual) {
			manualUpdate(s);
		} else {
			autoUpdate(s);
		}

		StateReportUpdate(s);
	}
}