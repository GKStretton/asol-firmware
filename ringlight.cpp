#include "ringlight.h"
#include "config.h"
#include <Arduino.h>

void RingLight::Toggle() {
    digitalWrite(LIGHT_TOGGLE, LOW);
    delay(LIGHT_BUTTON_WAIT_MS);
    digitalWrite(LIGHT_TOGGLE, HIGH);

    delay(500);

    // Mode twice gets to brighter white
    digitalWrite(LIGHT_MODE, LOW);
    delay(LIGHT_BUTTON_WAIT_MS);
    digitalWrite(LIGHT_MODE, HIGH);

    delay(500);

    digitalWrite(LIGHT_MODE, LOW);
    delay(LIGHT_BUTTON_WAIT_MS);
    digitalWrite(LIGHT_MODE, HIGH);
}