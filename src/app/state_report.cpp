#include "state_report.h"
#include <Arduino.h>
#include "../protos/machinepb/machine.pb.h"
#include "../protos/nanopb/pb_encode.h"
#include "../middleware/serialmqtt.h"
#include "../middleware/logger.h"

void StateReportUpdate(State *s) {
	_machine_PingResponse resp = {
		.number = 5,
	};
	uint8_t buf[10];
	Logger::Debug("created message and buffer");
	pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
	Logger::Debug("created stream");
	pb_encode(&stream, machine_PingResponse_fields, &resp);

	Logger::Debug("encoded");
	SerialMQTT::PublishMega("state-report", String((char *) buf));
	Logger::Debug("sent");
}