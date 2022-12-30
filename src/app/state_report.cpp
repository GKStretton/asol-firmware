#include "state_report.h"
#include <Arduino.h>
#include "../protos/nanopb/pb_encode.h"
#include "../middleware/serialmqtt.h"
#include "../middleware/logger.h"

static machine_StateReport stateReport = machine_StateReport_init_default;
// set to true if a state report should be sent out next time
static bool hasChanged = false;

// Ensures everything that isn't set through public functions gets updated.
// Will set hasChanged = true if something changed.
static void updateStateReport(State *s) {
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
	if (stateReport.movement_details.target_ring_deg != s->target_ring) {
		stateReport.movement_details.target_ring_deg = s->target_ring;
		hasChanged = true;
	}
	if (stateReport.movement_details.target_yaw_deg != s->target_yaw) {
		stateReport.movement_details.target_yaw_deg = s->target_yaw;
		hasChanged = true;
	}
}

void StateReport_Update(State *s) {
	updateStateReport(s);
	Logger::Debug("updateStateReport completed");
	if (hasChanged) {
		SerialMQTT::PublishProto("state-report", machine_PipetteState_fields, &(stateReport.pipette_state));
		hasChanged = false;
	}
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