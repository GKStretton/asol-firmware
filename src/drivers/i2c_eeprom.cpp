#include "i2c_eeprom.h"
#include "../config.h"
#include "../middleware/logger.h"
#include <Wire.h>

void I2C_EEPROM::WriteByte(int addr, uint8_t data) {
	Wire.beginTransmission(EEPROM_I2C_ADDRESS);
	Wire.write((int)(addr >> 8)); // MSB
	Wire.write((int)(addr & 0xFF)); // LSB
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t I2C_EEPROM::ReadByte(int addr) {
	uint8_t readData = 0xFF;

	Logger::Debug("begin transmission");
	Wire.beginTransmission(EEPROM_I2C_ADDRESS);
	Logger::Debug("writing");
	Wire.write((int)(addr >> 8)); // MSB
	Wire.write((int)(addr & 0xFF)); // LSB
	Logger::Debug("end transmission");
	Wire.endTransmission();

	Logger::Debug("request");
	Wire.requestFrom(EEPROM_I2C_ADDRESS, 1);
	if (Wire.available()) {
		Logger::Debug("read");
		readData = Wire.read();
	}
	Logger::Debug("done");
	return readData;
}