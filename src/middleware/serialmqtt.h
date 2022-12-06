#pragma once

#include <Arduino.h>

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
	// Unpacks n values separated by commas into values[]. Returns true if error
	void UnpackCommaSeparatedValues(String payload, String values[], int n);
};