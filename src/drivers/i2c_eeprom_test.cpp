#include "i2c_eeprom.h"
#include "../middleware/logger.h"
#include "../config.h"

int I2C_EEPROM::Test() {
	float num = 0.123;
	unsigned long start = micros();
	WriteFloat(FLUID_LEVEL_TEST_ADDR, num);
	// Note it takes time behind the scenes to do the actual write
	unsigned long writeTime = micros() - start;
	delay(5);

	start = micros();
	float result = ReadFloat(FLUID_LEVEL_TEST_ADDR);
	unsigned long readTime = micros() - start;

	if (num != result) {
		Logger::Error("FAIL: Expected " + String(num) + " but got " + String(result));
		return 1;
	}
	Logger::Info("PASS: eeprom write time: " + String(writeTime) + "us. eeprom read time: " + String(readTime) + "us");

	return 0;
}