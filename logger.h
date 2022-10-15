#pragma once
#include <Arduino.h>

namespace Logger {

    enum Level {
        NONE,
        WARN,
        INFO,
        DEBUG,
        VERBOSE,
    };

    void SetLevel(enum Level level);

    void Warn(String str);
    void Info(String str);
    void Debug(String str);
    void Verbose(String str);

    void PrintDataEntry(String str, String val);

    void TestLogger();
};