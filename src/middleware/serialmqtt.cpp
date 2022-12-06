#include "serialmqtt.h"
#include "../config.h"
#include "../middleware/logger.h"

char buffer[SERIAL_MQTT_BUFFER_SIZE];
int bufferIndex = 0;

void (*TopicHandler)(String topic, String payload) = NULL;

void SerialMQTT::SetTopicHandler(void (*f)(String topic, String payload)) {
	TopicHandler = f;
}

void processInputBuffer() {
	String topic = "";
	int ptr = 0;

	// Get the topic, everything up to ';'
	while (true) {
		if (ptr >= bufferIndex) {
			Logger::Error("Out of buffer length without delimiter (shouldn't happen)");
			// Reset buffer
			bufferIndex = 0;
			return;
		}
		char c = buffer[ptr];
		if (c == '\n') {
			Logger::Error("New line before topic delimiter");
			// Reset buffer
			bufferIndex = 0;
			return;
		}

		ptr++;
		if (c == ';') {
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
		Logger::Warn("topic handler is null, mqtt serial will not be handled");
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
	Serial.print('>');
	Serial.print(SERIAL_MQTT_SEND_PREFIX);
	Serial.print(topic);
	Serial.print(";");
	Serial.println(payload);
}

void SerialMQTT::PublishRawTopic(String topic, String payload) {
	Serial.print('>');
	Serial.print(topic);
	Serial.print(";");
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