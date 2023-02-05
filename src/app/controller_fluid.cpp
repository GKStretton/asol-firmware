#include "controller.h"
#include "../middleware/logger.h"
#include "../drivers/i2c_eeprom.h"
#include "../config.h"
#include "../calibration.h"
#include "../common/util.h"

static float readFluidLevel() {
	return I2C_EEPROM::ReadFloat(FLUID_LEVEL_FLOAT_ADDR);
}

static void writeFluidLevel(float level) {
	I2C_EEPROM::WriteFloat(FLUID_LEVEL_FLOAT_ADDR, level);
	Logger::Info("Set fluid level to " + String(level));
}

// converts fluid volume (ml) into valve open time (ms)
static float getValveOpenTimeFromVolume(FluidType t, float volume_ml) {
	if (t == FluidType::FLUID_UNDEFINED) {
		return 0;
	} else if (t == FluidType::DRAIN) {
		return (volume_ml / DRAIN_VOLUME_PER_SECOND_ML) * 1000.0;
	} else {
		return (volume_ml / DISPENSE_VOLUME_PER_SECOND_ML) * 1000.0;
	}
}

static uint8_t pinFromFluidType(FluidType t) {
	if (t == FluidType::DRAIN) {
		return DRAINAGE_VALVE_RELAY;
	} else if (t == FluidType::MILK) {
		return MILK_VALVE_RELAY;
	} else if (t == FluidType::WATER) {
		return WATER_VALVE_RELAY;
	} else {
		return AIR_VALVE_RELAY;
	}
}

static void setValve(uint8_t pin, bool open) {
	SetDualRelay(pin, open);
}

void Controller::fluidInit(State *s) {
}

void Controller::fluidUpdate(State *s) {
	if (s->fluidRequest.complete) {
		return;
	}
	// we have an unfilled state change

	uint8_t pin = pinFromFluidType(s->fluidRequest.fluidType);
	float openTime = getValveOpenTimeFromVolume(s->fluidRequest.fluidType, s->fluidRequest.volume_ml);

	// start, open the valve
	if (s->fluidRequest.startTime == 0) {
		s->fluidRequest.startTime = millis();
		setValve(pin, true);
	}

	if (millis() - s-> fluidRequest.startTime >= openTime) {
		setValve(pin, false);
		setValve(AIR_VALVE_RELAY, true);
	}

	if (millis() - s-> fluidRequest.startTime >= openTime + FLUID_TRAVEL_TIME_MS) {
		setValve(AIR_VALVE_RELAY, false);
		s->fluidRequest.complete = true;
		if (s->fluidRequest.fluidType == FluidType::DRAIN) {
			float level = readFluidLevel();
			writeFluidLevel(level - s->fluidRequest.volume_ml);
		}
	}
}

void Controller::NewFluidRequest(State *s, FluidType fluidType, float volume_ml) {
	if (readFluidLevel() + volume_ml > MAX_FLUID_LEVEL) {
		Logger::Warn("fluid level would exceed maximum, rejecting request");
		return;
	}
	s->fluidRequest.fluidType = fluidType;
	s->fluidRequest.volume_ml = volume_ml;
	s->fluidRequest.startTime = 0;
	s->fluidRequest.complete = false;
	Logger::Info("Set fluid request type " +
		String(fluidType) + " volume " + String(volume_ml));

	if (s->fluidRequest.fluidType != FluidType::FLUID_UNDEFINED &&
		s->fluidRequest.fluidType != FluidType::DRAIN)
	{
		float level = readFluidLevel();
		writeFluidLevel(level + s->fluidRequest.volume_ml);
	}
}