#include "fluid_levels.h"
#include "../drivers/i2c_eeprom.h"
#include "../middleware/logger.h"
#include "../config.h"

const float CACHE_NIL_VALUE = -54321.5;
float fluidLevelCache;

float FluidLevels_ReadBowlLevel() {
	if (fluidLevelCache != CACHE_NIL_VALUE) {
		return fluidLevelCache;
	}
	fluidLevelCache = I2C_EEPROM::ReadFloat(FLUID_LEVEL_FLOAT_ADDR);
	return fluidLevelCache;
}

void FluidLevels_WriteBowlLevel(float level) {
	fluidLevelCache = level;
	I2C_EEPROM::WriteFloat(FLUID_LEVEL_FLOAT_ADDR, level);
	Logger::Info("Set fluid level to " + String(level));
}
