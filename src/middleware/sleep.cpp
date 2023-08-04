#include "sleep.h"
#include <Arduino.h>
#include "../common/util.h"
#include "../config.h"
#include "../middleware/serialmqtt.h"
#include "../middleware/logger.h"
#include "../drivers/ringlight.h"
#include "../drivers/i2c_eeprom.h"
#include "../app/state_report.h"
#include "../config.h"
#include "../extras/topics_backend/topics_backend.h"

namespace Sleep {
	namespace {
		unsigned long lastNod = millis();
		unsigned long lastPrint = millis() - SLEEP_PRINT_INTERVAL;
		bool sleeping = true;
		SleepStatus lastSleepStatus = SleepStatus::UNKNOWN;

		void (*externalSleepHandler)(SleepStatus sleepStatus) = NULL;
		void (*externalWakeHandler)(SleepStatus lastSleepStatus) = NULL;

		bool eStopActive() {
			return digitalRead(E_STOP_PIN) == LOW;
		}

		void onSleep(SleepStatus status) {
			Logger::Info("Going to sleep with status " + String(status));

			// write eeprom flag
			I2C_EEPROM::WriteByte(SAFE_SHUTDOWN_EEPROM_FLAG_ADDR, (int) status);

			if (externalSleepHandler != NULL) {
				Logger::Info("Calling externalSleepHandler");
				externalSleepHandler(status);
			}

			delay(200);

			Logger::Info("Powering down.");
			SetDualRelay(V12_RELAY_PIN1, false);
			SetDualRelay(V12_RELAY_PIN2, false);

			delay(500);

			SetDualRelay(V5_RELAY_PIN, false);

			delay(500);

			// turn off power with smart switch
			SerialMQTT::Publish(TOPIC_SMART_SWITCH, PAYLOAD_SMART_SWITCH_OFF);
			Logger::Info("External power off req sent.");

			if (eStopActive()) {
				StateReport_SetStatus(machine_Status_E_STOP_ACTIVE);
			} else {
				StateReport_SetStatus(machine_Status_SLEEPING);
			}
			StateReport_ForceSend();
		}

		void onWake() {
			StateReport_SetStatus(machine_Status_WAKING_UP);
			StateReport_ForceSend();

			Logger::Info("Waking up, powering up");
			SerialMQTT::Publish(TOPIC_SMART_SWITCH, PAYLOAD_SMART_SWITCH_ON);
			delay(2000);
			SetDualRelay(V5_RELAY_PIN, true);
			SetDualRelay(V12_RELAY_PIN1, true);
			SetDualRelay(V12_RELAY_PIN2, true);
			delay(1000);

			uint8_t data = I2C_EEPROM::ReadByte(SAFE_SHUTDOWN_EEPROM_FLAG_ADDR);
			lastSleepStatus = (SleepStatus) data;
			Logger::Info("read lastSleepStatus as " + String(lastSleepStatus));
			// write back to unknown in case of sudden shutdown
			I2C_EEPROM::WriteByte(SAFE_SHUTDOWN_EEPROM_FLAG_ADDR, (int) UNKNOWN);

			if (externalWakeHandler != NULL) {
				Logger::Info("Calling externalWakeHandler");
				externalWakeHandler(lastSleepStatus);
			}
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
			Sleep(UNKNOWN);
			if (eStopActive()) {
				StateReport_SetStatus(machine_Status_E_STOP_ACTIVE);
			} else {
				StateReport_SetStatus(machine_Status_SLEEPING);
			}
		} else {
			Wake();
		}
		
		if (millis() - lastPrint > SLEEP_PRINT_INTERVAL) {
			lastPrint = millis();
			if (eStopActive()) {
				Logger::Info("E_STOP ACTIVE");
			} else if (sleeping) {
				Logger::Info("sleeping");
			}
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

	void Sleep(SleepStatus status) {
		bool wasSleeping = sleeping;
		sleeping = true;
		if (!wasSleeping) {
			onSleep(status);
		}
	}

	bool IsSleeping() {
		return sleeping;
	}

	bool IsEStopActive() {
		return eStopActive();
	}

	SleepStatus GetLastSleepStatus() {
		return lastSleepStatus;
	}

	void OverrideLastSleepStatus(SleepStatus status) {
		lastSleepStatus = status;
	}

	void SetOnSleepHandler(void (*f)(SleepStatus)) {
		externalSleepHandler = f;
	}

	void SetOnWakeHandler(void (*f)(SleepStatus)) {
		externalWakeHandler = f;
	}
}