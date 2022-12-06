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
			SerialMQTT::PublishMega("l/error", str);
		}
	}

	void Warn(String str) {
		if (level >= WARN) {
			SerialMQTT::PublishMega("l/warn", str);
		}
	}

	void Info(String str) {
		if (level >= INFO) {
			SerialMQTT::PublishMega("l/info", str);
		}
	}

	void Debug(String str) {
		if (level >= DEBUG) {
			SerialMQTT::PublishMega("l/debug", str);
		}
	}
}