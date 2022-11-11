#include "sleep.h"
#include <Arduino.h>
#include "util.h"
#include "config.h"
#include "logger.h"
#include "ringlight.h"

namespace Sleep {
	namespace {
		unsigned long lastNod = millis();
		bool sleeping = true;

		void onSleep() {
			Logger::Info("Going to sleep");
			SetDualRelay(V5_RELAY_PIN, false);
			SetDualRelay(V12_RELAY_PIN1, false);
			SetDualRelay(V12_RELAY_PIN2, false);
		}

		void onWake() {
			Logger::Info("Waking up");
			SetDualRelay(V5_RELAY_PIN, true);
			SetDualRelay(V12_RELAY_PIN1, true);
			SetDualRelay(V12_RELAY_PIN2, true);
			delay(100);
			RingLight::Toggle();
		}
	}

	void Update() {
		if (digitalRead(BUTTON_A)) {
			Wake();
		}

		bool previous = sleeping;
		if (SLEEP_TIME_MINUTES > 0 && (millis() - lastNod) / 1000 / 60 >= SLEEP_TIME_MINUTES) {
			sleeping = true;
		}
		if (!previous && sleeping) { // Just went asleep
			onSleep();
		}
	}

	void Wake() {
		lastNod = millis();
		bool wasSleeping = sleeping;
		sleeping = false;
		if (wasSleeping) {
			onWake();
		}
	}

	bool IsSleeping() {
		return sleeping;
	}

	void Sleep() {
		sleeping = true;
		onSleep();
	}
}