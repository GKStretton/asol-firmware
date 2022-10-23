#include "ringlight.h"
#include "config.h"
#include "util.h"
#include <Arduino.h>

void setLightPin(uint8_t pin, bool state) {
    // digitalWrite(pin, state ? LOW: HIGH);
    SetDualRelay(pin, state);
}

void RingLight::Toggle() {
    setLightPin(LIGHT_TOGGLE, true);
    delay(LIGHT_BUTTON_WAIT_MS);
    setLightPin(LIGHT_TOGGLE, false);

    delay(500);

    // Mode twice gets to brighter white
    setLightPin(LIGHT_MODE, true);
    delay(LIGHT_BUTTON_WAIT_MS);
    setLightPin(LIGHT_MODE, false);

    delay(500);

    setLightPin(LIGHT_MODE, true);
    delay(LIGHT_BUTTON_WAIT_MS);
    setLightPin(LIGHT_MODE, false);
}