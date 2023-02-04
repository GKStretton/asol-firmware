#pragma once

#include <Arduino.h>
#include "../protos/nanopb/pb_encode.h"

#define SERIAL_MQTT_BUFFER_SIZE 256
#define SERIAL_MQTT_SEND_PREFIX "mega/"
#define SERIAL_MQTT_MESSAGE_START '>'
#define SERIAL_MQTT_PLAINTEXT_IDENTIFIER '#'
#define SERIAL_MQTT_PROTOBUF_IDENTIFIER '$'
#define SERIAL_MQTT_TOPIC_END ';'
#define SERIAL_MQTT_MESSAGE_END '\n'

namespace SerialMQTT {
	// The function that will be called upon receipt of topic & payload
	void SetTopicHandler(void (*f)(String topic, String payload));
	// Process Serial input and call handler if \n is reached
	void Update();
	// PublishMega a payload to a topic via SerialMQTT with mega prefix
	void PublishMega(String topic, String payload);
	// Publish to a raw topic without mega prefix
	void PublishRawTopic(String topic, String payload);

	// Payload helpers
	// Unpacks n values separated by commas into values[].
	void UnpackCommaSeparatedValues(String payload, String values[], int n);

	// Publish a nanopb proto to the specified topic (mega prefix will be added)
	void PublishProto(String topic, const pb_msgdesc_t *fields, const void *src_struct);
};