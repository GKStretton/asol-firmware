#include "fluid.h"
#include "../common/util.h"
#include "../config.h"

void SetupFluid() {
	InitPin(WATER_VALVE_RELAY, HIGH);
	InitPin(MILK_VALVE_RELAY, HIGH);
	InitPin(AIR_VALVE_RELAY, HIGH);
}

void SetAirFlow(bool flow) {
    SetDualRelay(AIR_VALVE_RELAY, flow);
}

void SetFlushFlow(bool flow) {
    SetDualRelay(WATER_VALVE_RELAY, flow);
}

void SetCanvasFlow(bool flow) {
    SetDualRelay(MILK_VALVE_RELAY, flow);
}