#include "logger.h"

namespace Logger {

    namespace {
        enum Level level = DEBUG;
    }

    void SetLevel(enum Level _level) {
        level = _level;
    }

    void Warn(String str) {
        if (level >= WARN) {
            Serial.print("[WARN] ");
            Serial.println(str);
        }
    }

    void Info(String str) {
        if (level >= INFO) {
            Serial.print("[INFO] ");
            Serial.println(str);
        }
    }

    void Debug(String str) {
        if (level >= DEBUG) {
            Serial.print("[DEBUG] ");
            Serial.println(str);
        }
    }

    void Verbose(String str) {
        if (level >= VERBOSE) {
            Serial.print("[VERB] ");
            Serial.println(str);
        }
    }

    void PrintDataEntry(String str, String val) {
        Serial.println(">" + str + "\t" + val);
    }
}