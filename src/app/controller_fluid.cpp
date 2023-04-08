#include "controller.h"
#include "../middleware/logger.h"
#include "../drivers/i2c_eeprom.h"
#include "../config.h"
#include "../calibration.h"
#include "../common/util.h"
#include "../middleware/fluid_levels.h"

// converts fluid volume (ml) into valve open time (ms)
static float getValveOpenTimeFromVolume(FluidType t, float volume_ml) {
	if (t == FluidType::FLUID_UNDEFINED) {
		return 0;
	} else if (t == FluidType::DRAIN) {
		return (volume_ml / DRAIN_VOLUME_PER_SECOND_ML) * 1000.0;
	} else if (t == FluidType::WATER) {
		return (volume_ml / WATER_VOLUME_PER_SECOND_ML) * 1000.0;
	} else if (t == FluidType::MILK) {
		return (volume_ml / MILK_VOLUME_PER_SECOND_ML) * 1000.0;
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
	// time to drain the requested amount of fluid
	float drainTime = getValveOpenTimeFromVolume(FluidType::DRAIN, s->fluidRequest.volume_ml);

	// start, open the valve
	if (s->fluidRequest.startTime == 0) {
		s->fluidRequest.startTime = millis();
		setValve(pin, true);
	}

	// do open_drain after a delay
	if (s->fluidRequest.open_drain &&
		millis() - s->fluidRequest.startTime >= OPEN_DRAIN_DELAY_MS + FLUID_TRAVEL_TIME_MS)
	{
		setValve(DRAINAGE_VALVE_RELAY, true);
	}


	if (millis() - s->fluidRequest.startTime >= openTime) {
		setValve(pin, false);

		if (s->fluidRequest.fluidType == FluidType::DRAIN) {
			s->fluidRequest.complete = true;
			float newLevel = FluidLevels_ReadBowlLevel() - s->fluidRequest.volume_ml;
			if (newLevel < 0) {
				newLevel = 0;
			}
			FluidLevels_WriteBowlLevel(newLevel);
			return;
		}

		setValve(AIR_VALVE_RELAY, true);
	}

	if (millis() - s->fluidRequest.startTime >= openTime + FLUID_TRAVEL_TIME_MS) {
		setValve(AIR_VALVE_RELAY, false);
		if (!s->fluidRequest.open_drain) {
			s->fluidRequest.complete = true;
			return;
		}
	}

	if (s->fluidRequest.open_drain &&
		millis() - s->fluidRequest.startTime >= FLUID_TRAVEL_TIME_MS + drainTime + OPEN_DRAIN_DELAY_MS)
	{
		setValve(DRAINAGE_VALVE_RELAY, false);
		s->fluidRequest.complete = true;
	}
}

void Controller::NewFluidRequest(State *s, FluidType fluidType, float volume_ml, bool open_drain) {
	if (fluidType == FluidType::FLUID_UNDEFINED) {
		Logger::Warn("undefined fluidType request");
		return;
	}
	float fluidLevel = FluidLevels_ReadBowlLevel();
	if (fluidType != FluidType::DRAIN && fluidLevel + volume_ml > MAX_FLUID_LEVEL) {
		Logger::Warn("fluid level would exceed maximum, rejecting new request");
		return;
	}
	if (!s->fluidRequest.complete) {
		Logger::Warn("current fluid request not complete, rejecting new request");
		return;
	}
	s->fluidRequest.fluidType = fluidType;
	s->fluidRequest.volume_ml = volume_ml;
	s->fluidRequest.startTime = 0;
	s->fluidRequest.complete = false;
	s->fluidRequest.open_drain = open_drain;
	Logger::Info("Set fluid request type " +
		String(fluidType) + " volume " + String(volume_ml));

	if (s->fluidRequest.fluidType != FluidType::FLUID_UNDEFINED &&
		s->fluidRequest.fluidType != FluidType::DRAIN &&
		!s->fluidRequest.open_drain)
	{
		FluidLevels_WriteBowlLevel(fluidLevel + s->fluidRequest.volume_ml);
	}
}