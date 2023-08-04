#include "state_report.h"
#include <Arduino.h>
#include "../extras/nanopb/pb_encode.h"
#include "../middleware/serialmqtt.h"
#include "../middleware/logger.h"
#include "../middleware/fluid_levels.h"
#include "../extras/topics_firmware/topics_firmware.h"

static machine_StateReport stateReport = machine_StateReport_init_default;
// set to true if a state report should be sent out next time
static bool hasChanged = false;

// Ensures everything that isn't set through public functions gets updated.
// Will set hasChanged = true if something changed.
static void updateStateReport(State *s) {
	if (stateReport.startup_counter != s->startup_counter) {
		stateReport.startup_counter = s->startup_counter;
		hasChanged = true;
	}

	/*
		pipette_state
	*/
	stateReport.has_pipette_state = true;
	if (stateReport.pipette_state.spent != s->pipetteState.spent) {
		stateReport.pipette_state.spent = s->pipetteState.spent;
		hasChanged = true;
	}
	if (stateReport.pipette_state.vial_held != s->pipetteState.vialHeld) {
		stateReport.pipette_state.vial_held = s->pipetteState.vialHeld;
		hasChanged = true;
	}
	if (stateReport.pipette_state.volume_target_ul != s->pipetteState.ulVolumeHeldTarget) {
		stateReport.pipette_state.volume_target_ul = s->pipetteState.ulVolumeHeldTarget;
		hasChanged = true;
	}
	if (stateReport.pipette_state.dispense_request_number != s->pipetteState.dispenseRequestNumber) {
		stateReport.pipette_state.dispense_request_number = s->pipetteState.dispenseRequestNumber;
		hasChanged = true;
	}

	/*
		collection_request
	*/
	stateReport.has_collection_request = true;
	if (stateReport.collection_request.completed != s->collectionRequest.requestCompleted) {
		stateReport.collection_request.completed = s->collectionRequest.requestCompleted;
		hasChanged = true;
	}
	if (stateReport.collection_request.request_number != s->collectionRequest.requestNumber) {
		stateReport.collection_request.request_number = s->collectionRequest.requestNumber;
		hasChanged = true;
	}
	if (stateReport.collection_request.vial_number != s->collectionRequest.vialNumber) {
		stateReport.collection_request.vial_number = s->collectionRequest.vialNumber;
		hasChanged = true;
	}
	if (stateReport.collection_request.volume_ul != s->collectionRequest.ulVolume) {
		stateReport.collection_request.volume_ul = s->collectionRequest.ulVolume;
		hasChanged = true;
	}

	/*
		movement_details
	*/
	stateReport.has_movement_details = true;
	if (stateReport.movement_details.target_x_unit != s->target_x) {
		stateReport.movement_details.target_x_unit = s->target_x;
		hasChanged = true;
	}
	if (stateReport.movement_details.target_y_unit != s->target_y) {
		stateReport.movement_details.target_y_unit = s->target_y;
		hasChanged = true;
	}
	if (stateReport.movement_details.target_z_ik != s->ik_target_z) {
		stateReport.movement_details.target_z_ik = s->ik_target_z;
		hasChanged = true;
	}
	if (stateReport.movement_details.target_ring_deg != s->target_ring) {
		stateReport.movement_details.target_ring_deg = s->target_ring;
		hasChanged = true;
	}
	if (stateReport.movement_details.target_yaw_deg != s->target_yaw) {
		stateReport.movement_details.target_yaw_deg = s->target_yaw;
		hasChanged = true;
	}

	/*
		fluid_request
	*/
	stateReport.has_fluid_request = true;
	if (stateReport.fluid_request.fluidType != s->fluidRequest.fluidType) {
		stateReport.fluid_request.fluidType = (_machine_FluidType) s->fluidRequest.fluidType;
		hasChanged = true;
	}
	if (stateReport.fluid_request.open_drain != s->fluidRequest.open_drain) {
		stateReport.fluid_request.open_drain = s->fluidRequest.open_drain;
		hasChanged = true;
	}
	if (stateReport.fluid_request.volume_ml != s->fluidRequest.volume_ml) {
		stateReport.fluid_request.volume_ml = s->fluidRequest.volume_ml;
		hasChanged = true;
	}
	if (stateReport.fluid_request.complete != s->fluidRequest.complete) {
		stateReport.fluid_request.complete = s->fluidRequest.complete;
		hasChanged = true;
	}

	/*
		fluid_details
	*/
	stateReport.has_fluid_details = true;
	float bowlLevel = FluidLevels_ReadBowlLevel();
	if (stateReport.fluid_details.bowl_fluid_level_ml != bowlLevel) {
		stateReport.fluid_details.bowl_fluid_level_ml = bowlLevel;
		hasChanged = true;
	}

	if (stateReport.rinse_status != s->rinseStatus) {
		stateReport.rinse_status = s->rinseStatus;
		hasChanged = true;
	}
}

void StateReport_Update(State *s) {
	updateStateReport(s);
	if (hasChanged) {
		StateReport_ForceSend();
	}
}

void StateReport_ForceSend() {
	SerialMQTT::PublishProto(TOPIC_STATE_REPORT_RAW, machine_StateReport_fields, &stateReport);
	hasChanged = false;
}

/*
********************* PUBLIC MANUAL SETTERS ******************
*/

void StateReport_SetMode(machine_Mode mode) {
	if (stateReport.mode != mode) {
		stateReport.mode = mode;
		hasChanged = true;
	}
}

void StateReport_SetStatus(machine_Status status) {
	if (stateReport.status != status) {
		stateReport.status = status;
		hasChanged = true;
	}
}

void StateReport_SetLights(bool on) {
	if (stateReport.lights_on != on) {
		stateReport.lights_on = on;
		hasChanged = true;
	}
}