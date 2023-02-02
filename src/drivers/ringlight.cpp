#include "ringlight.h"
#include <Arduino.h>
#include "../config.h"
#include "../common/util.h"

void setLightPin(uint8_t pin, bool state) {
	// digitalWrite(pin, state ? LOW: HIGH);
	SetDualRelay(pin, state);
}

void RingLight::Toggle() {
	setLightPin(TOP_LIGHT_TOGGLE, true);
	setLightPin(FRONT_LIGHT_TOGGLE, true);
	delay(LIGHT_BUTTON_WAIT_MS);
	setLightPin(TOP_LIGHT_TOGGLE, false);
	setLightPin(FRONT_LIGHT_TOGGLE, false);

	delay(500);

	// Mode twice gets to brighter white
	setLightPin(TOP_LIGHT_MODE, true);
	delay(LIGHT_BUTTON_WAIT_MS);
	setLightPin(TOP_LIGHT_MODE, false);

	delay(500);

	setLightPin(TOP_LIGHT_MODE, true);
	delay(LIGHT_BUTTON_WAIT_MS);
	setLightPin(TOP_LIGHT_MODE, false);
}