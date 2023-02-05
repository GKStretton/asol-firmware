#include "testing.h"
#include "../middleware/logger.h"

#include "../drivers/i2c_eeprom.h"

int Testing_RunTests() {
	Logger::Info("--- STARTING TESTS ---");
	int eepromResult = I2C_EEPROM::Test();
	if (eepromResult) {
		Logger::Error("EEPROM test failed");
		return eepromResult;
	}
	Logger::Info("--- All tests passed ---");
	return 0;
}