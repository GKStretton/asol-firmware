#include "logger.h"

namespace Logger {
    namespace {
        void multiprint(String str) {
            Info(str);
            Debug(str);
            Verbose(str);
        }
    }

    void TestLogger() {
        Serial.println();
        Serial.println("*** logger_test.cpp ***");
        Serial.println();

        for (int i = NONE; i <= VERBOSE; i++) {
            Serial.println("Level " + String(i));
            SetLevel((Level) i);
            multiprint(String(i));
        }

        Serial.println();

        PrintDataEntry("apples", String(5));
        PrintDataEntry("bananas", String(7.5));
        PrintDataEntry("blueberries", String(52.2));

        Serial.println();

        PrintDataEntry("ape", "meme");
        PrintDataEntry("banano", String(5));
        PrintDataEntry("bluebs", String(2.2));

        Serial.println();

        PrintDataEntry("ape", "meme");
        PrintDataEntry("banano", String(5));
        PrintDataEntry("bluebs", String(2.2));

        Serial.println();

        for (int i = NONE; i <= VERBOSE; i++) {
            Serial.println("Level " + String(i));
            SetLevel((Level) i);
            multiprint(String(i));
            Serial.println();
        }

    }
}