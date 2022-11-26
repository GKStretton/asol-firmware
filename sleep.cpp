#include "sleep.h"
#include <Arduino.h>
#include "util.h"
#include "config.h"
#include "logger.h"
#include "ringlight.h"

namespace Sleep {
	namespace {
		unsigned long lastNod = millis();
		unsigned long lastPrint = millis();
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

		bool eStopActive() {
			return digitalRead(E_STOP_PIN) == LOW;
		}

		// private version of isSleeping that does the actual checks
		bool isSleeping() {
			//! ordered by priority
			if (eStopActive()) {
				return true;
			}
			
			if (digitalRead(BUTTON_A)) {
				return false;
			}

			if (SLEEP_TIME_MINUTES > 0 && (millis() - lastNod) / 1000 / 60 >= SLEEP_TIME_MINUTES) {
				return true;
			}
			
			// persist current state by default
			return sleeping;
		}
	}

	void Update() {
		if (isSleeping()) {
			Sleep();
		} else {
			Wake();
		}
		if (millis() - lastPrint > SLEEP_PRINT_INTERVAL) {
			if (eStopActive()) {
				Logger::Info("E_STOP ACTIVE");
			} else if (sleeping) {
				Logger::Info("sleeping");
			}
			lastPrint = millis();
		}
	}

	void Wake() {
		if (eStopActive()) {
			return;
		}

		lastNod = millis();
		bool wasSleeping = sleeping;
		sleeping = false;
		if (wasSleeping) {
			onWake();
		}
	}

	void Sleep() {
		bool wasSleeping = sleeping;
		sleeping = true;
		if (!wasSleeping) {
			onSleep();
		}
	}

	bool IsSleeping() {
		return sleeping;
	}
}