#pragma once
#include <Arduino.h>

// Ensure wire.begin is called before anything in this namespace.
namespace I2C_EEPROM {
	// Write a byte to the specified memory address
	void WriteByte(int addr, uint8_t data);
	// Read a byte from the specified memory address
	uint8_t ReadByte(int addr);
	void WriteFloat(int addr, float data);
	float ReadFloat(int addr);

	int Test();
};