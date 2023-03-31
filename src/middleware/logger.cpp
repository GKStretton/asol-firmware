#include "logger.h"
#include "../middleware/serialmqtt.h"
#include "../extras/topics_firmware/topics_firmware.h"

namespace Logger {

	namespace {
		enum Level level = DEBUG;
	}

	void SetLevel(enum Level _level) {
		level = _level;
	}

	void Error(String str) {
		if (level >= ERROR) {
			SerialMQTT::Publish(TOPIC_LOGS_ERROR, str);
		}
	}

	void Warn(String str) {
		if (level >= WARN) {
			SerialMQTT::Publish(TOPIC_LOGS_WARN, str);
		}
	}

	void Info(String str) {
		if (level >= INFO) {
			SerialMQTT::Publish(TOPIC_LOGS_INFO, str);
		}
	}

	void Debug(String str) {
		if (level >= DEBUG) {
			SerialMQTT::Publish(TOPIC_LOGS_DEBUG, str);
		}
	}
}