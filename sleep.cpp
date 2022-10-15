#include "sleep.h"
#include <Arduino.h>
#include "util.h"
#include "config.h"
#include "logger.h"
#include "ringlight.h"

namespace Sleep {
	namespace {
		unsigned long lastNod = millis();
		bool sleeping = false;

		void onSleep() {
			Logger::Info("Sleeping");
			SetDualRelay(V5_RELAY_PIN, false);
		}

		void onWake() {
			Logger::Info("Waking up");
			SetDualRelay(V5_RELAY_PIN, true);
			delay(100);
			RingLight::Toggle();
		}
	}

	void Update() {
		if (digitalRead(BUTTON_A)) {
			Wake();
		}

		bool previous = sleeping;
		sleeping = (millis() - lastNod) / 1000 / 60 >= SLEEP_TIME_MINUTES;
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
}