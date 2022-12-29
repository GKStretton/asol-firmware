#include "state_report.h"
#include <Arduino.h>
#include "../protos/machinepb/machine.pb.h"
#include "../protos/nanopb/pb_encode.h"
#include "../middleware/serialmqtt.h"
#include "../middleware/logger.h"

static machine_StateReport stateReport = machine_StateReport_init_default;

// returns true if something changed
static bool updateStateReport(State *s) {
	bool changed = false;
	//todo: implement

	// init all fields

	// for each...
	// if stateReport.field != s.field {
	//     stateReport.field = s.field;
	//     changed = true;
	// }
	return false;
}

void StateReportUpdate(State *s) {
	if (updateStateReport(s)) {
		SerialMQTT::PublishProto("state-report", machine_PingResponse_fields, &stateReport);
	}
}