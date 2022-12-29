#include "state_report.h"
#include <Arduino.h>
#include "../protos/machinepb/machine.pb.h"
#include "../protos/nanopb/pb_encode.h"
#include "../middleware/serialmqtt.h"
#include "../middleware/logger.h"

void StateReportUpdate(State *s) {
	_machine_PingResponse resp = {
		.number = 55,
	};
	SerialMQTT::PublishProto("state-report", machine_PingResponse_fields, &resp);
}