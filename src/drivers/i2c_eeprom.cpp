#include "i2c_eeprom.h"
#include "../config.h"
#include "../middleware/logger.h"
#include <Wire.h>

typedef unsigned char byte;

void I2C_EEPROM::WriteByte(int addr, uint8_t data) {
	Logger::Info("Sending to eeprom for WRITE. (If hanging, check voltages)");
	Wire.beginTransmission(EEPROM_I2C_ADDRESS);
	Wire.write((int)(addr >> 8)); // MSB
	Wire.write((int)(addr & 0xFF)); // LSB
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t I2C_EEPROM::ReadByte(int addr) {
	uint8_t readData = 0xFF;

	Logger::Info("Sending to eeprom for READ. (If hanging, check voltages)");
	Wire.beginTransmission(EEPROM_I2C_ADDRESS);
	Wire.write((int)(addr >> 8)); // MSB
	Wire.write((int)(addr & 0xFF)); // LSB
	Wire.endTransmission();

	Wire.requestFrom(EEPROM_I2C_ADDRESS, 1);
	if (Wire.available()) {
		readData = Wire.read();
	}
	return readData;
}

void I2C_EEPROM::WriteFloat(int addr, float data) {
	byte* bytes = (byte*) &data;

	Logger::Debug("Writing float to eeprom");
	Wire.beginTransmission(EEPROM_I2C_ADDRESS);
	Wire.write((int)(addr >> 8)); // MSB
	Wire.write((int)(addr & 0xFF)); // LSB
	for (int i = 0; i < 4; i++) {
		Wire.write(bytes[i]);
	}
	Wire.endTransmission();
}

float I2C_EEPROM::ReadFloat(int addr) {
	float result;
	byte* bytes = (byte*) &result;

	Logger::Debug("Reading float from eeprom");
	Wire.beginTransmission(EEPROM_I2C_ADDRESS);
	Wire.write((int)(addr >> 8)); // MSB
	Wire.write((int)(addr & 0xFF)); // LSB
	Wire.endTransmission();

	Wire.requestFrom(EEPROM_I2C_ADDRESS, 4);
	for (int i = 0; i < 4; i++) {
		if (Wire.available()) {
			bytes[i] = Wire.read();
		}
	}
	return result;
}