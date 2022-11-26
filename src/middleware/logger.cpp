#include "logger.h"
#include "../middleware/serialmqtt.h"

namespace Logger {

	namespace {
		enum Level level = DEBUG;
	}

	void SetLevel(enum Level _level) {
		level = _level;
	}

	void Error(String str) {
		if (level >= ERROR) {
			SerialMQTT::Publish("l/error", str);
		}
	}

	void Warn(String str) {
		if (level >= WARN) {
			SerialMQTT::Publish("l/warn", str);
		}
	}

	void Info(String str) {
		if (level >= INFO) {
			SerialMQTT::Publish("l/info", str);
		}
	}

	void Debug(String str) {
		if (level >= DEBUG) {
			SerialMQTT::Publish("l/debug", str);
		}
	}
}