#pragma once
#include <Arduino.h>

namespace Logger {

	enum Level {
		NONE,
		ERROR,
		WARN,
		INFO,
		DEBUG,
	};

	void SetLevel(enum Level level);

	void Error(String str);
	void Warn(String str);
	void Info(String str);
	void Debug(String str);

	void TestLogger();
};