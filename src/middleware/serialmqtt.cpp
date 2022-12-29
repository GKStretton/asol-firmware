#include "serialmqtt.h"
#include "../config.h"
#include "../protos/pb_arduino/pb_arduino.h"

char buffer[SERIAL_MQTT_BUFFER_SIZE];
int bufferIndex = 0;

void (*TopicHandler)(String topic, String payload) = NULL;

void SerialMQTT::SetTopicHandler(void (*f)(String topic, String payload)) {
	TopicHandler = f;
}

void processInputBuffer() {
	String topic = "";
	int ptr = 0;
	
	// Get the topic, everything up to SERIAL_MQTT_PLAINTEXT_IDENTIFIER
	while (true) {
		if (ptr >= bufferIndex) {
			Serial.println("Out of buffer length without delimiter (shouldn't happen)");
			// Reset buffer
			bufferIndex = 0;
			return;
		}
		char c = buffer[ptr];
		if (c == '\n') {
			Serial.println("New line before topic delimiter");
			// Reset buffer
			bufferIndex = 0;
			return;
		}

		ptr++;
		if (c == SERIAL_MQTT_DELIMITER) {
			break;
		} else {
			topic += c;
		}
	}

	// Get the payload, everything else up to '\n'
	String payload = "";
	while (ptr < bufferIndex) {
		char c = buffer[ptr];
		if (c == '\n') {
			break;
		}
		payload += c;
		ptr++;
	}

	// reset buffer
	bufferIndex = 0;

	if (TopicHandler == NULL) {
		Serial.println("topic handler is null, mqtt serial will not be handled");
	} else {
		TopicHandler(topic, payload);
	}
}

void SerialMQTT::Update() {
	if (Serial.available() > 0) {
		int c = Serial.read();
		buffer[bufferIndex++] = (char) c;
		if (c == '\n') {
			processInputBuffer();
		}
	}
}

void SerialMQTT::PublishMega(String topic, String payload) {
	Serial.print(SERIAL_MQTT_PLAINTEXT_IDENTIFIER);
	Serial.print(SERIAL_MQTT_SEND_PREFIX);
	Serial.print(topic);
	Serial.print(SERIAL_MQTT_DELIMITER);
	Serial.println(payload);
}

void SerialMQTT::PublishRawTopic(String topic, String payload) {
	Serial.print(SERIAL_MQTT_PLAINTEXT_IDENTIFIER);
	Serial.print(topic);
	Serial.print(SERIAL_MQTT_DELIMITER);
	Serial.println(payload);
}

void SerialMQTT::UnpackCommaSeparatedValues(String payload, String values[], int n) {
	// Keeps track of which comma separated value we're on
	int value_index = 0;
	for (int i = 0; i < payload.length(); i++) {
		// Return if all values are found
		if (value_index >= n) return;

		// Go to next value index
		if (payload[i] == ',') {
			values[++value_index] = "";
			continue;
		}

		// append character to current value
		values[value_index] += payload[i];
	}
	// return error if we didn't find exactly n comma separated values
	return;
}

void SerialMQTT::PublishProto(String topic, const pb_msgdesc_t *fields, const void *src_struct) {
	Serial.print(SERIAL_MQTT_PROTOBUF_IDENTIFIER);
	Serial.print(SERIAL_MQTT_SEND_PREFIX);
	Serial.print(topic);
	Serial.print(SERIAL_MQTT_DELIMITER);
	
	//? should this be created only once?
	pb_ostream_t stream = as_pb_ostream(Serial);
	bool b = pb_encode(&stream, fields, src_struct);
	Serial.println();
	if (!b) {
		Serial.println("pb_encode return false in PublishProto");
	}
}