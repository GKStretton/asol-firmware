#include "logger.h"

namespace Logger {
	namespace {
		void multiprint(String str) {
			Error(str);
			Warn(str);
			Info(str);
			Debug(str);
		}
	}

	void TestLogger() {
		Serial.println();
		Serial.println("*** logger_test.cpp ***");
		Serial.println();

		for (int i = NONE; i <= DEBUG; i++) {
			Serial.println("Level " + String(i));
			SetLevel((Level) i);
			multiprint(String(i));
			Serial.println();
		}
	}
}