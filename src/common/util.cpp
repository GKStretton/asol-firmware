#include "util.h"

void InitPin(uint8_t pin, byte v) {
	pinMode(pin, OUTPUT);
	digitalWrite(pin, v);
}

void Bound(int *number, int minimum, int maximum) {
	if (*number < minimum) {
		*number = minimum;
	}
	if (*number > maximum) {
		*number = maximum;
	}
}

// setDualRelay operates on 2 channel relays that
// turn on by sending low signal.
void SetDualRelay(uint8_t pin, bool on) {
	digitalWrite(pin, on ? LOW : HIGH);
}

// setSingleRelay operates on single channel relays
// that turn on by sending high signal.
void SetSingleRelay(uint8_t pin, bool on) {
	digitalWrite(pin, on ? HIGH : LOW);
}